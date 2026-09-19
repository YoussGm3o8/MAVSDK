#include "mavsdk.hpp"
#include "plugins/geofence/geofence.hpp"
#include "plugins/mavlink_passthrough/mavlink_passthrough.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

using namespace mavsdk;

namespace {

Geofence::GeofenceData make_fence(Geofence::FenceType type)
{
    Geofence::Polygon polygon{};
    polygon.fence_type = type;
    polygon.points = {{42.3908, -71.1486}, {42.3908, -71.1466}, {42.3888, -71.1466}};
    Geofence::GeofenceData data{};
    data.polygons.push_back(polygon);
    return data;
}

class ArduPilotFenceFaults : public ::testing::Test {
protected:
    void SetUp() override
    {
        const auto* url = std::getenv("MAVSDK_ARDUPILOT_URL");
        if (url == nullptr || std::string{url}.empty()) {
            GTEST_SKIP() << "set MAVSDK_ARDUPILOT_URL to run the ArduPilot SITL suite";
        }
        sdk = std::make_unique<Mavsdk>(Mavsdk::Configuration{ComponentType::GroundStation});
        ASSERT_EQ(sdk->add_any_connection(url), ConnectionResult::Success);
        system = sdk->first_autopilot(20.0).value_or(nullptr);
        ASSERT_NE(system, nullptr);
        ASSERT_EQ(system->autopilot_type(), Autopilot::ArduPilot);
        geofence = std::make_unique<Geofence>(system);
        ASSERT_EQ(geofence->clear_geofence(), Geofence::Result::Success);
    }

    void TearDown() override
    {
        if (geofence) {
            EXPECT_EQ(geofence->clear_geofence(), Geofence::Result::Success);
        }
    }

    void verify_round_trip(const Geofence::GeofenceData& expected)
    {
        ASSERT_EQ(geofence->upload_geofence(expected), Geofence::Result::Success);
        const auto [result, actual] = geofence->download_geofence();
        ASSERT_EQ(result, Geofence::Result::Success);
        ASSERT_EQ(actual.polygons.size(), expected.polygons.size());
        ASSERT_EQ(actual.polygons.front().points.size(), expected.polygons.front().points.size());
        EXPECT_EQ(actual.polygons.front().fence_type, expected.polygons.front().fence_type);
        for (size_t index = 0; index < expected.polygons.front().points.size(); ++index) {
            const auto& point = actual.polygons.front().points[index];
            const auto& wanted = expected.polygons.front().points[index];
            EXPECT_NEAR(point.latitude_deg, wanted.latitude_deg, 1e-7);
            EXPECT_NEAR(point.longitude_deg, wanted.longitude_deg, 1e-7);
        }
        EXPECT_TRUE(actual.circles.empty());
    }

    std::unique_ptr<Mavsdk> sdk;
    std::shared_ptr<System> system;
    std::unique_ptr<Geofence> geofence;
};

TEST_F(ArduPilotFenceFaults, RoundTripsExclusionPolygon)
{
    verify_round_trip(make_fence(Geofence::FenceType::Exclusion));
}

TEST_F(ArduPilotFenceFaults, ReportsVehicleRejectedUploadAndRecovers)
{
    auto rejected_ack = std::make_shared<std::atomic<unsigned>>(0);
    MavlinkPassthrough wire{system};
    const auto handle = wire.subscribe_message(
        MAVLINK_MSG_ID_MISSION_ACK, [rejected_ack](const mavlink_message_t& message) {
            mavlink_mission_ack_t ack{};
            mavlink_msg_mission_ack_decode(&message, &ack);
            if (ack.mission_type == MAV_MISSION_TYPE_FENCE && ack.type == MAV_MISSION_NO_SPACE) {
                ++*rejected_ack;
            }
        });
    auto data = make_fence(Geofence::FenceType::Inclusion);
    // This exceeds the pinned Copter's fence storage, but is valid MAVLink input.
    data.polygons.front().points.resize(256, data.polygons.front().points.front());
    EXPECT_EQ(geofence->upload_geofence(data), Geofence::Result::TooManyGeofenceItems);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (rejected_ack->load() == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    wire.unsubscribe_message(MAVLINK_MSG_ID_MISSION_ACK, handle);
    EXPECT_EQ(rejected_ack->load(), 1U) << "the rejection must come from ArduCopter";
    verify_round_trip(make_fence(Geofence::FenceType::Inclusion));
}

TEST_F(ArduPilotFenceFaults, UploadTimesOutDuringResponseLossAndRecovers)
{
    auto lost_requests = std::make_shared<std::atomic<unsigned>>(0);
    const auto handle =
        sdk->subscribe_incoming_messages_json([lost_requests](Mavsdk::MavlinkMessage message) {
            if (message.message_name == "MISSION_REQUEST_INT" ||
                message.message_name == "MISSION_REQUEST") {
                ++*lost_requests;
                return false;
            }
            return true;
        });
    sdk->set_timeout_s(0.2);
    const auto start = std::chrono::steady_clock::now();
    const auto result = geofence->upload_geofence(make_fence(Geofence::FenceType::Inclusion));
    const auto elapsed = std::chrono::steady_clock::now() - start;
    sdk->unsubscribe_incoming_messages_json(handle);
    sdk->set_timeout_s(0.5);
    EXPECT_EQ(result, Geofence::Result::Timeout);
    EXPECT_GT(lost_requests->load(), 0U) << "must lose a response from the real vehicle";
    EXPECT_GE(elapsed, std::chrono::milliseconds(200));
    EXPECT_LT(elapsed, std::chrono::seconds(5));
    verify_round_trip(make_fence(Geofence::FenceType::Inclusion));
}

TEST_F(ArduPilotFenceFaults, DownloadTimesOutDuringResponseLossAndRecovers)
{
    const auto data = make_fence(Geofence::FenceType::Inclusion);
    ASSERT_EQ(geofence->upload_geofence(data), Geofence::Result::Success);
    auto lost_counts = std::make_shared<std::atomic<unsigned>>(0);
    const auto handle =
        sdk->subscribe_incoming_messages_json([lost_counts](Mavsdk::MavlinkMessage message) {
            if (message.message_name == "MISSION_COUNT") {
                ++*lost_counts;
                return false;
            }
            return true;
        });
    sdk->set_timeout_s(0.2);
    const auto start = std::chrono::steady_clock::now();
    const auto [result, incomplete] = geofence->download_geofence();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    sdk->unsubscribe_incoming_messages_json(handle);
    sdk->set_timeout_s(0.5);
    EXPECT_EQ(result, Geofence::Result::Timeout);
    EXPECT_GT(lost_counts->load(), 1U) << "must observe retries against the real vehicle";
    EXPECT_TRUE(incomplete.polygons.empty());
    EXPECT_TRUE(incomplete.circles.empty());
    EXPECT_GE(elapsed, std::chrono::milliseconds(200));
    EXPECT_LT(elapsed, std::chrono::seconds(5));
    verify_round_trip(data);
}

} // namespace

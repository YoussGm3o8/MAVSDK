#include "mavsdk.hpp"
#include "plugins/mavlink_passthrough/mavlink_passthrough.hpp"
#include "plugins/offboard/offboard.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace mavsdk;
using namespace std::chrono_literals;

namespace {

struct ReceivedSetpoints {
    std::mutex mutex;
    std::vector<mavlink_set_position_target_local_ned_t> messages;
};

class OffboardOneShot : public ::testing::Test {
protected:
    void SetUp() override
    {
        const auto [result, handle] =
            client.add_any_connection_with_handle("udpin://0.0.0.0:17934");
        ASSERT_EQ(result, ConnectionResult::Success);
        connection = handle;
        ASSERT_EQ(peer.add_any_connection("udpout://127.0.0.1:17934"), ConnectionResult::Success);
        auto vehicle = client.first_autopilot(5.0).value_or(nullptr);
        ASSERT_NE(vehicle, nullptr);
        offboard = std::make_unique<Offboard>(vehicle);
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (peer.systems().empty() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(20ms);
        }
        ASSERT_FALSE(peer.systems().empty());
        wire = std::make_unique<MavlinkPassthrough>(peer.systems().front());
        wire->subscribe_message(
            MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
            [state = received](const mavlink_message_t& message) {
                mavlink_set_position_target_local_ned_t value{};
                mavlink_msg_set_position_target_local_ned_decode(&message, &value);
                std::lock_guard lock(state->mutex);
                state->messages.push_back(value);
            });
    }

    std::vector<mavlink_set_position_target_local_ned_t> get_messages()
    {
        std::lock_guard lock(received->mutex);
        return received->messages;
    }

    bool wait_for_count(size_t count)
    {
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (get_messages().size() >= count) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

    Mavsdk client{Mavsdk::Configuration{ComponentType::GroundStation}};
    Mavsdk peer{Mavsdk::Configuration{ComponentType::Autopilot}};
    Mavsdk::ConnectionHandle connection{};
    std::unique_ptr<Offboard> offboard;
    std::unique_ptr<MavlinkPassthrough> wire;
    std::shared_ptr<ReceivedSetpoints> received = std::make_shared<ReceivedSetpoints>();
};

TEST_F(OffboardOneShot, EncodesBodyVelocityWithoutResendingWhenProducerStalls)
{
    ASSERT_EQ(
        offboard->set_velocity_body_once({1.0F, 2.0F, -0.5F, 90.0F}), Offboard::Result::Success);
    ASSERT_TRUE(wait_for_count(1));
    const auto message = get_messages().front();
    EXPECT_EQ(message.target_system, 1);
    EXPECT_EQ(message.target_component, MAV_COMP_ID_AUTOPILOT1);
    EXPECT_EQ(message.coordinate_frame, MAV_FRAME_BODY_NED);
    EXPECT_EQ(message.type_mask, 1479);
    EXPECT_FLOAT_EQ(message.vx, 1.0F);
    EXPECT_FLOAT_EQ(message.vy, 2.0F);
    EXPECT_FLOAT_EQ(message.vz, -0.5F);
    EXPECT_NEAR(message.yaw_rate, 1.5707963F, 1e-6F);
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(get_messages().size(), 1U) << "a stalled producer must not leave a resend timer";
    EXPECT_FALSE(offboard->is_active());
}

TEST_F(OffboardOneShot, ZeroAndDestructionLeaveNoPendingResends)
{
    ASSERT_EQ(
        offboard->set_velocity_body_once({1.0F, 0.0F, 0.0F, 0.0F}), Offboard::Result::Success);
    ASSERT_TRUE(wait_for_count(1));
    ASSERT_EQ(offboard->set_velocity_body_once({}), Offboard::Result::Success);
    ASSERT_TRUE(wait_for_count(2));
    offboard.reset();
    std::this_thread::sleep_for(400ms);
    const auto messages = get_messages();
    ASSERT_EQ(messages.size(), 2U);
    EXPECT_FLOAT_EQ(messages.back().vx, 0.0F);
    EXPECT_FLOAT_EQ(messages.back().vy, 0.0F);
    EXPECT_FLOAT_EQ(messages.back().vz, 0.0F);
    EXPECT_FLOAT_EQ(messages.back().yaw_rate, 0.0F);
}

TEST_F(OffboardOneShot, RejectsNonfiniteComponentsWithoutSending)
{
    for (const auto invalid :
         {std::numeric_limits<float>::quiet_NaN(),
          std::numeric_limits<float>::infinity(),
          -std::numeric_limits<float>::infinity()}) {
        EXPECT_EQ(offboard->set_velocity_body_once({invalid, 0, 0, 0}), Offboard::Result::Failed);
        EXPECT_EQ(offboard->set_velocity_body_once({0, invalid, 0, 0}), Offboard::Result::Failed);
        EXPECT_EQ(offboard->set_velocity_body_once({0, 0, invalid, 0}), Offboard::Result::Failed);
        EXPECT_EQ(offboard->set_velocity_body_once({0, 0, 0, invalid}), Offboard::Result::Failed);
    }
    std::this_thread::sleep_for(200ms);
    EXPECT_TRUE(get_messages().empty());
}

TEST_F(OffboardOneShot, ReportsRemovedConnectionWithoutRetainingSetpoint)
{
    client.remove_connection(connection);
    EXPECT_EQ(offboard->set_velocity_body_once({1, 0, 0, 0}), Offboard::Result::ConnectionError);
    EXPECT_FALSE(offboard->is_active());
    std::this_thread::sleep_for(200ms);
    EXPECT_TRUE(get_messages().empty());
}

TEST_F(OffboardOneShot, RejectsMixingWithAutomaticResends)
{
    ASSERT_EQ(offboard->set_velocity_body({0.5F, 0, 0, 0}), Offboard::Result::Success);
    ASSERT_TRUE(wait_for_count(1));
    EXPECT_EQ(offboard->set_velocity_body_once({1.0F, 0, 0, 0}), Offboard::Result::Busy);
    ASSERT_TRUE(wait_for_count(2));
    for (const auto& message : get_messages()) {
        EXPECT_FLOAT_EQ(message.vx, 0.5F);
    }
}

} // namespace

#include "mavsdk.hpp"
#include "plugins/geofence/geofence.hpp"

#include <chrono>
#include <memory>

#include <gtest/gtest.h>

using namespace mavsdk;

TEST(GeofenceTransport, TimesOutWhenAutopilotIgnoresFenceDownload)
{
    Mavsdk client{Mavsdk::Configuration{ComponentType::GroundStation}};
    Mavsdk silent_autopilot{Mavsdk::Configuration{ComponentType::Autopilot}};
    client.set_timeout_s(0.2);

    ASSERT_EQ(client.add_any_connection("udpin://0.0.0.0:17931"), ConnectionResult::Success);
    ASSERT_EQ(
        silent_autopilot.add_any_connection("udpout://127.0.0.1:17931"), ConnectionResult::Success);

    auto system = client.first_autopilot(5.0).value_or(nullptr);
    ASSERT_NE(system, nullptr);
    Geofence geofence(system);

    const auto started = std::chrono::steady_clock::now();
    const auto [result, data] = geofence.download_geofence();
    const auto elapsed = std::chrono::steady_clock::now() - started;

    EXPECT_EQ(result, Geofence::Result::Timeout);
    EXPECT_TRUE(data.polygons.empty());
    EXPECT_TRUE(data.circles.empty());
    EXPECT_GE(elapsed, std::chrono::milliseconds(200));
    EXPECT_LT(elapsed, std::chrono::seconds(10));
}

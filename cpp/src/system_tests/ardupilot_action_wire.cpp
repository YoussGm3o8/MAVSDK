#include "mavsdk.hpp"
#include "plugins/action/action.hpp"
#include "plugins/mavlink_passthrough/mavlink_passthrough.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <gtest/gtest.h>

using namespace mavsdk;

namespace {

std::shared_ptr<System> wait_for_ground_station(Mavsdk& autopilot, uint8_t ground_station_id)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        for (const auto& system : autopilot.systems()) {
            if (system->is_connected() && system->get_system_id() == ground_station_id) {
                return system;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return nullptr;
}

void acknowledge_command(MavlinkPassthrough& passthrough, const mavlink_message_t& request, uint16_t command)
{
    passthrough.queue_message([request, command](MavlinkAddress address, uint8_t channel) {
        mavlink_message_t acknowledgment{};
        mavlink_msg_command_ack_pack_chan(
            address.system_id,
            address.component_id,
            channel,
            &acknowledgment,
            command,
            MAV_RESULT_ACCEPTED,
            0,
            0,
            request.sysid,
            request.compid);
        return acknowledgment;
    });
}

} // namespace

TEST(ActionTransport, SendsRelativeRepositionAsCommandInt)
{
    Mavsdk::Configuration client_config{ComponentType::GroundStation};
    const auto ground_station_id = client_config.get_system_id();
    Mavsdk client{client_config};
    Mavsdk autopilot{Mavsdk::Configuration{ComponentType::Autopilot}};
    ASSERT_EQ(client.add_any_connection("udpin://0.0.0.0:17932"), ConnectionResult::Success);
    ASSERT_EQ(autopilot.add_any_connection("udpout://127.0.0.1:17932"), ConnectionResult::Success);

    const auto vehicle = client.first_autopilot(5.0).value_or(nullptr);
    ASSERT_NE(vehicle, nullptr);
    const auto ground_station = wait_for_ground_station(autopilot, ground_station_id);
    ASSERT_NE(ground_station, nullptr);

    MavlinkPassthrough passthrough{ground_station};
    std::atomic<unsigned> gcs_heartbeats{0};
    const auto heartbeat_handle = passthrough.subscribe_message(
        MAVLINK_MSG_ID_HEARTBEAT, [&](const mavlink_message_t& message) {
            if (message.sysid == ground_station_id) {
                ++gcs_heartbeats;
            }
        });
    const auto heartbeat_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (gcs_heartbeats.load() == 0 && std::chrono::steady_clock::now() < heartbeat_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    passthrough.unsubscribe_message(MAVLINK_MSG_ID_HEARTBEAT, heartbeat_handle);
    ASSERT_GT(gcs_heartbeats.load(), 0U);

    std::mutex command_mutex;
    std::optional<mavlink_command_int_t> reposition;
    std::atomic<unsigned> long_repositions{0};

    const auto long_handle = passthrough.subscribe_message(
        MAVLINK_MSG_ID_COMMAND_LONG, [&](const mavlink_message_t& message) {
            mavlink_command_long_t command{};
            mavlink_msg_command_long_decode(&message, &command);
            if (command.command == MAV_CMD_DO_REPOSITION) {
                ++long_repositions;
            }
            acknowledge_command(passthrough, message, command.command);
        });
    const auto int_handle = passthrough.subscribe_message(
        MAVLINK_MSG_ID_COMMAND_INT, [&](const mavlink_message_t& message) {
            mavlink_command_int_t command{};
            mavlink_msg_command_int_decode(&message, &command);
            if (command.command == MAV_CMD_DO_REPOSITION) {
                std::lock_guard lock(command_mutex);
                reposition = command;
            }
            acknowledge_command(passthrough, message, command.command);
        });

    Action action{vehicle};
    constexpr double latitude_deg = 42.3898123;
    constexpr double longitude_deg = -71.1476543;
    constexpr float relative_altitude_m = 17.5F;
    const auto result = action.goto_location_relative(
        latitude_deg,
        longitude_deg,
        relative_altitude_m,
        45.0F,
        OperationOptions{std::chrono::seconds(3)});

    passthrough.unsubscribe_message(MAVLINK_MSG_ID_COMMAND_INT, int_handle);
    passthrough.unsubscribe_message(MAVLINK_MSG_ID_COMMAND_LONG, long_handle);
    ASSERT_EQ(result, Action::Result::Success);
    EXPECT_EQ(long_repositions.load(), 0U);
    std::lock_guard lock(command_mutex);
    ASSERT_TRUE(reposition.has_value());
    EXPECT_EQ(reposition->command, MAV_CMD_DO_REPOSITION);
    EXPECT_EQ(reposition->frame, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    EXPECT_EQ(reposition->x, 423898123);
    EXPECT_EQ(reposition->y, -711476543);
    EXPECT_FLOAT_EQ(reposition->z, relative_altitude_m);
    EXPECT_FLOAT_EQ(reposition->param2, MAV_DO_REPOSITION_FLAGS_CHANGE_MODE);
    EXPECT_NEAR(reposition->param4, 0.78539816F, 0.000001F);
}

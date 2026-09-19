#include "mavsdk.hpp"
#include "plugins/mavlink_passthrough/mavlink_passthrough.hpp"
#include "plugins/param/param.hpp"
#include "plugins/param_server/param_server.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

using namespace mavsdk;
using namespace std::chrono_literals;

namespace {

constexpr uint16_t test_command = MAV_CMD_USER_1;

void send_ack(MavlinkPassthrough& wire, MAV_RESULT result)
{
    wire.queue_message([result](MavlinkAddress address, uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_command_ack_pack_chan(
            address.system_id,
            address.component_id,
            channel,
            &message,
            test_command,
            result,
            50,
            0,
            245,
            MAV_COMP_ID_MISSIONPLANNER);
        return message;
    });
}

class OperationDeadlineWire : public ::testing::Test {
protected:
    void SetUp() override
    {
        ASSERT_EQ(client.add_any_connection("udpin://0.0.0.0:17933"), ConnectionResult::Success);
        ASSERT_EQ(peer.add_any_connection("udpout://127.0.0.1:17933"), ConnectionResult::Success);
        vehicle = client.first_autopilot(5.0).value_or(nullptr);
        ASSERT_NE(vehicle, nullptr);
        command = std::make_unique<MavlinkPassthrough>(vehicle);
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (peer.systems().empty() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(20ms);
        }
        ASSERT_FALSE(peer.systems().empty());
        wire = std::make_unique<MavlinkPassthrough>(peer.systems().front());
    }

    MavlinkPassthrough::CommandLong make_command() const
    {
        MavlinkPassthrough::CommandLong value{};
        value.target_sysid = 1;
        value.target_compid = MAV_COMP_ID_AUTOPILOT1;
        value.command = test_command;
        return value;
    }

    Mavsdk client{Mavsdk::Configuration{ComponentType::GroundStation}};
    Mavsdk peer{Mavsdk::Configuration{ComponentType::Autopilot}};
    std::shared_ptr<System> vehicle;
    std::unique_ptr<MavlinkPassthrough> command;
    std::unique_ptr<MavlinkPassthrough> wire;
};

TEST_F(OperationDeadlineWire, RetriesDroppedCommandThenSucceedsWithinBudget)
{
    auto attempts = std::make_shared<std::atomic<unsigned>>(0);
    const auto handle = wire->subscribe_message(
        MAVLINK_MSG_ID_COMMAND_LONG,
        [attempts, responder = wire.get()](const mavlink_message_t& message) {
            mavlink_command_long_t request{};
            mavlink_msg_command_long_decode(&message, &request);
            if (request.command == test_command && ++*attempts > 1) {
                send_ack(*responder, MAV_RESULT_ACCEPTED);
            }
        });
    const auto start = std::chrono::steady_clock::now();
    const auto result = command->send_command_long(make_command(), OperationOptions{800ms});
    const auto elapsed = std::chrono::steady_clock::now() - start;
    wire->unsubscribe_message(MAVLINK_MSG_ID_COMMAND_LONG, handle);
    EXPECT_EQ(result, MavlinkPassthrough::Result::Success);
    EXPECT_EQ(attempts->load(), 2U);
    EXPECT_GE(elapsed, 100ms);
    EXPECT_LT(elapsed, 1s);
}

TEST_F(OperationDeadlineWire, ProgressAcknowledgementsCannotExtendDeadline)
{
    auto received_progress = std::make_shared<std::atomic<unsigned>>(0);
    const auto ack_handle = command->subscribe_message(
        MAVLINK_MSG_ID_COMMAND_ACK, [received_progress](const mavlink_message_t& message) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&message, &ack);
            if (ack.command == test_command && ack.result == MAV_RESULT_IN_PROGRESS) {
                ++*received_progress;
            }
        });
    auto attempts = std::make_shared<std::atomic<unsigned>>(0);
    const auto handle = wire->subscribe_message(
        MAVLINK_MSG_ID_COMMAND_LONG, [attempts](const mavlink_message_t& message) {
            mavlink_command_long_t request{};
            mavlink_msg_command_long_decode(&message, &request);
            if (request.command == test_command) {
                ++*attempts;
            }
        });
    auto progress_count = std::make_shared<std::atomic<unsigned>>(0);
    std::jthread progress([attempts, progress_count, responder = wire.get()](std::stop_token stop) {
        while (!stop.stop_requested()) {
            if (attempts->load() != 0) {
                send_ack(*responder, MAV_RESULT_IN_PROGRESS);
                ++*progress_count;
            }
            std::this_thread::sleep_for(20ms);
        }
    });
    const auto start = std::chrono::steady_clock::now();
    auto operation = std::async(std::launch::async, [&] {
        return command->send_command_long(make_command(), OperationOptions{400ms});
    });
    const auto completion = operation.wait_for(800ms);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    progress.request_stop();
    progress.join();
    command->unsubscribe_message(MAVLINK_MSG_ID_COMMAND_ACK, ack_handle);
    wire->unsubscribe_message(MAVLINK_MSG_ID_COMMAND_LONG, handle);
    EXPECT_EQ(completion, std::future_status::ready);
    EXPECT_EQ(operation.get(), MavlinkPassthrough::Result::CommandTimeout);
    EXPECT_GT(progress_count->load(), 5U);
    EXPECT_GT(received_progress->load(), 5U);
    EXPECT_EQ(attempts->load(), 1U);
    EXPECT_GE(elapsed, 350ms);
    EXPECT_LT(elapsed, 800ms);
}

TEST_F(OperationDeadlineWire, ConcurrentCommandsKeepIndependentDeadlines)
{
    auto attempts = std::make_shared<std::atomic<unsigned>>(0);
    auto short_attempts = std::make_shared<std::atomic<unsigned>>(0);
    const auto handle = wire->subscribe_message(
        MAVLINK_MSG_ID_COMMAND_LONG, [attempts, short_attempts](const mavlink_message_t& message) {
            mavlink_command_long_t request{};
            mavlink_msg_command_long_decode(&message, &request);
            if (request.command == test_command) {
                ++*attempts;
            } else if (request.command == MAV_CMD_USER_2) {
                ++*short_attempts;
            }
        });
    const auto start = std::chrono::steady_clock::now();
    auto longer = std::async(std::launch::async, [&] {
        return command->send_command_long(make_command(), OperationOptions{900ms});
    });
    const auto deadline = start + 300ms;
    while (attempts->load() == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(5ms);
    }
    EXPECT_GT(attempts->load(), 0U);
    const auto short_start = std::chrono::steady_clock::now();
    auto shorter = make_command();
    shorter.command = MAV_CMD_USER_2;
    EXPECT_EQ(
        command->send_command_long(shorter, OperationOptions{200ms}),
        MavlinkPassthrough::Result::CommandTimeout);
    EXPECT_GT(short_attempts->load(), 0U) << "both commands must have been in flight";
    EXPECT_LT(std::chrono::steady_clock::now() - short_start, 500ms);
    EXPECT_EQ(longer.wait_for(0ms), std::future_status::timeout);
    EXPECT_EQ(longer.get(), MavlinkPassthrough::Result::CommandTimeout);
    EXPECT_GE(std::chrono::steady_clock::now() - start, 800ms);
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1400ms);
    wire->unsubscribe_message(MAVLINK_MSG_ID_COMMAND_LONG, handle);
}

TEST_F(OperationDeadlineWire, RetriesDroppedParameterReadThenReturnsValue)
{
    ParamServer server{peer.server_component()};
    ASSERT_EQ(server.provide_param_float("RETRY_VALUE", 37.5F), ParamServer::Result::Success);
    auto attempts = std::make_shared<std::atomic<unsigned>>(0);
    const auto handle =
        peer.subscribe_incoming_messages_json([attempts](Mavsdk::MavlinkMessage message) {
            if (message.message_name == "PARAM_REQUEST_READ") {
                return ++*attempts > 1;
            }
            return true;
        });
    Param param{vehicle};
    const auto start = std::chrono::steady_clock::now();
    const auto [result, value] = param.get_param_float("RETRY_VALUE", OperationOptions{900ms});
    const auto elapsed = std::chrono::steady_clock::now() - start;
    peer.unsubscribe_incoming_messages_json(handle);
    EXPECT_EQ(result, Param::Result::Success);
    EXPECT_FLOAT_EQ(value, 37.5F);
    EXPECT_EQ(attempts->load(), 2U);
    EXPECT_GE(elapsed, 100ms);
    EXPECT_LT(elapsed, 1s);
}

} // namespace

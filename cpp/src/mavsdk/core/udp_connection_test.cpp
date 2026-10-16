// SPDX-License-Identifier: BSD-3-Clause
#include "udp_connection.hpp"
#include "mavsdk_impl.hpp"
#include "mavlink_include.hpp"

#include <asio/post.hpp>
#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <thread>

namespace mavsdk {

struct UdpConnectionTestAccess {
    static std::shared_future<void> receive_stopped(UdpConnection& connection)
    {
        return connection._receive_stopped_future;
    }

    struct Snapshot {
        bool open;
        bool pending;
        bool stopping;
        bool completed;
        asio::ip::udp::endpoint endpoint;
    };

    static std::optional<Snapshot> inspect(UdpConnection& connection)
    {
        auto result = std::make_shared<std::promise<Snapshot>>();
        auto future = result->get_future();
        asio::post(connection.io_context(), [&connection, result] {
            asio::error_code ec;
            const auto endpoint = connection._socket.local_endpoint(ec);
            result->set_value(
                {connection._socket.is_open(),
                 connection._receive_pending,
                 connection._stopping,
                 connection._receive_stopped_future.wait_for(std::chrono::seconds(0)) ==
                     std::future_status::ready,
                 endpoint});
        });
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            return std::nullopt;
        }
        return future.get();
    }
};

TEST(UdpConnection, StopDrainsPendingReceiveBeforeDestruction)
{
    using namespace std::chrono_literals;
    for (const int payload_size : {-1, 0, 4096, 17}) {
        for (int iteration = 0; iteration < 10; ++iteration) {
            SCOPED_TRACE(
                ::testing::Message() << "payload=" << payload_size << " iteration=" << iteration);
            auto received = std::make_shared<std::atomic<unsigned>>(0);
            MavsdkImpl sdk{Mavsdk::Configuration{ComponentType::GroundStation}};
            auto connection = std::make_unique<UdpConnection>(
                [received](MavlinkReceiver::ParseResult result, mavlink_message_t&, Connection*) {
                    if (result == MavlinkReceiver::ParseResult::MessageParsed) {
                        ++*received;
                    }
                },
                [](const Mavsdk::MavlinkMessage&, Connection*) {},
                sdk,
                "127.0.0.1",
                0);
            ASSERT_EQ(connection->start(), ConnectionResult::Success);
            const auto before = UdpConnectionTestAccess::inspect(*connection);
            ASSERT_TRUE(before.has_value()) << "io thread did not answer the setup snapshot";
            ASSERT_TRUE(before->open);
            ASSERT_TRUE(before->pending);
            ASSERT_FALSE(before->stopping);
            ASSERT_FALSE(before->completed);
            asio::io_context peer_context;
            asio::ip::udp::socket peer{peer_context, asio::ip::udp::v4()};
            std::array<uint8_t, 4096> packet{};
            size_t length = payload_size > 0 ? static_cast<size_t>(payload_size) : 0;
            if (payload_size == 17) {
                mavlink_message_t heartbeat{};
                mavlink_msg_heartbeat_pack(
                    1,
                    1,
                    &heartbeat,
                    MAV_TYPE_QUADROTOR,
                    MAV_AUTOPILOT_ARDUPILOTMEGA,
                    0,
                    0,
                    MAV_STATE_ACTIVE);
                length = mavlink_msg_to_send_buffer(packet.data(), &heartbeat);
            }
            if (payload_size >= 0) {
                asio::error_code ec;
                peer.send_to(asio::buffer(packet.data(), length), before->endpoint, 0, ec);
                ASSERT_FALSE(ec) << "loopback datagram setup failed";
            }
            if (payload_size == 17) {
                const auto deadline = std::chrono::steady_clock::now() + 2s;
                while (received->load() == 0 && std::chrono::steady_clock::now() < deadline) {
                    std::this_thread::sleep_for(1ms);
                }
                ASSERT_GT(received->load(), 0U)
                    << "the real receive callback did not observe the heartbeat";
            }
            ASSERT_EQ(connection->stop(), ConnectionResult::Success);
            const auto stopped = UdpConnectionTestAccess::inspect(*connection);
            ASSERT_TRUE(stopped.has_value()) << "io thread did not answer after stop";
            EXPECT_FALSE(stopped->open);
            EXPECT_FALSE(stopped->pending);
            EXPECT_TRUE(stopped->stopping);
            EXPECT_TRUE(stopped->completed);
            ASSERT_EQ(connection->stop(), ConnectionResult::Success);
            connection.reset();
            asio::ip::udp::socket rebound{peer_context, asio::ip::udp::v4()};
            asio::error_code ec;
            rebound.bind(before->endpoint, ec);
            EXPECT_FALSE(ec) << "the stopped connection did not release its UDP port";
        }
    }
}

TEST(UdpConnection, SdkShutdownWithPendingReceiveFinishes)
{
    asio::io_context peer_context;
    asio::ip::udp::socket peer{
        peer_context, asio::ip::udp::endpoint{asio::ip::address_v4::loopback(), 0}};
    const auto endpoint = "udpout://127.0.0.1:" + std::to_string(peer.local_endpoint().port());
    const auto started = std::chrono::steady_clock::now();
    std::shared_future<void> receive_stopped;
    {
        MavsdkImpl sdk{Mavsdk::Configuration{ComponentType::GroundStation}};
        const auto result = sdk.add_any_connection(endpoint, ForwardingOption::ForwardingOff);
        ASSERT_EQ(result.first, ConnectionResult::Success);
        const auto connections = sdk.get_connections();
        ASSERT_EQ(connections.size(), 1U);
        auto& connection = static_cast<UdpConnection&>(*connections.front());
        const auto before = UdpConnectionTestAccess::inspect(connection);
        ASSERT_TRUE(before.has_value());
        ASSERT_TRUE(before->open);
        ASSERT_TRUE(before->pending);
        ASSERT_FALSE(before->completed);
        receive_stopped = UdpConnectionTestAccess::receive_stopped(connection);
    }
    EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::seconds(5));
    ASSERT_EQ(receive_stopped.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    EXPECT_NO_THROW(receive_stopped.get()) << "SDK shutdown abandoned the pending receive";
}

TEST(UdpConnection, StopBeforeStartAndAfterInvalidAddressFinishes)
{
    for (const bool invalid_address : {false, true}) {
        SCOPED_TRACE(invalid_address);
        MavsdkImpl sdk{Mavsdk::Configuration{ComponentType::GroundStation}};
        UdpConnection connection{
            [](MavlinkReceiver::ParseResult, mavlink_message_t&, Connection*) {},
            [](const Mavsdk::MavlinkMessage&, Connection*) {},
            sdk,
            "invalid-address",
            0};
        if (invalid_address) {
            ASSERT_EQ(connection.start(), ConnectionResult::SocketError);
        }
        ASSERT_EQ(connection.stop(), ConnectionResult::Success);
        const auto stopped = UdpConnectionTestAccess::inspect(connection);
        ASSERT_TRUE(stopped.has_value())
            << "io thread did not answer after a stop without a receive";
        EXPECT_FALSE(stopped->open);
        EXPECT_FALSE(stopped->pending);
        EXPECT_TRUE(stopped->stopping);
        EXPECT_TRUE(stopped->completed);
        EXPECT_EQ(connection.stop(), ConnectionResult::Success);
    }
}

} // namespace mavsdk

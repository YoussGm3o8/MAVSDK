#include "mavsdk.hpp"
#include "plugins/action/action.hpp"
#include "plugins/geofence/geofence.hpp"
#include "plugins/mavlink_passthrough/mavlink_passthrough.hpp"
#include "plugins/offboard/offboard.hpp"
#include "plugins/param/param.hpp"
#include "plugins/telemetry/telemetry.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <gtest/gtest.h>

using namespace mavsdk;

namespace {

constexpr auto kDiscoveryTimeout = 20.0;
constexpr auto kTelemetryTimeout = std::chrono::seconds(15);
constexpr auto kReadinessTimeout = std::chrono::seconds(30);
constexpr auto kOperationTimeout = std::chrono::milliseconds(300);
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::steady_clock::duration timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return predicate();
}

bool has_valid_coordinates(double latitude_deg, double longitude_deg)
{
    const bool is_in_range = std::isfinite(latitude_deg) && std::isfinite(longitude_deg) &&
                             latitude_deg >= -90.0 && latitude_deg <= 90.0 &&
                             longitude_deg >= -180.0 && longitude_deg <= 180.0;
    const bool is_not_placeholder = std::abs(latitude_deg) > 1e-6 || std::abs(longitude_deg) > 1e-6;
    return is_in_range && is_not_placeholder;
}

std::optional<std::string> ardupilot_url()
{
    const auto* value = std::getenv("MAVSDK_ARDUPILOT_URL");
    if (value == nullptr || std::string{value}.empty()) {
        return std::nullopt;
    }
    return std::string{value};
}

template<typename Value, typename Subscribe, typename Unsubscribe>
std::optional<Value> wait_for_value(Subscribe subscribe, Unsubscribe unsubscribe)
{
    auto promise = std::make_shared<std::promise<Value>>();
    auto future = promise->get_future();
    auto delivered = std::make_shared<std::atomic_bool>(false);
    const auto handle = subscribe([promise, delivered](Value value) {
        if (!delivered->exchange(true)) {
            promise->set_value(value);
        }
    });

    if (future.wait_for(kTelemetryTimeout) != std::future_status::ready) {
        unsubscribe(handle);
        return std::nullopt;
    }

    auto value = future.get();
    unsubscribe(handle);
    return value;
}

std::optional<Telemetry::Position> wait_for_valid_position(Telemetry& telemetry)
{
    auto promise = std::make_shared<std::promise<Telemetry::Position>>();
    auto future = promise->get_future();
    auto delivered = std::make_shared<std::atomic_bool>(false);
    const auto handle =
        telemetry.subscribe_position([promise, delivered](Telemetry::Position position) {
            const bool coordinates_are_valid =
                has_valid_coordinates(position.latitude_deg, position.longitude_deg);
            if (coordinates_are_valid && !delivered->exchange(true)) {
                promise->set_value(position);
            }
        });

    if (future.wait_for(kTelemetryTimeout) != std::future_status::ready) {
        telemetry.unsubscribe_position(handle);
        return std::nullopt;
    }

    auto position = future.get();
    telemetry.unsubscribe_position(handle);
    return position;
}

bool wait_for_guided_position_ready(Telemetry& telemetry)
{
    return wait_until(
        [&telemetry] {
            const auto position = telemetry.position();
            const auto home = telemetry.home();
            const bool position_is_valid =
                has_valid_coordinates(position.latitude_deg, position.longitude_deg);
            const bool home_is_valid = has_valid_coordinates(home.latitude_deg, home.longitude_deg);
            return position_is_valid && home_is_valid;
        },
        kReadinessTimeout);
}

bool wait_for_armable_position(Telemetry& telemetry)
{
    if (!wait_for_guided_position_ready(telemetry)) {
        return false;
    }
    return wait_until(
        [&telemetry] {
            const auto health = telemetry.health();
            return health.is_global_position_ok && health.is_armable;
        },
        std::chrono::seconds(60));
}

struct BodyDisplacement {
    double forward_m{};
    double right_m{};
};

BodyDisplacement
body_displacement(const Telemetry::Position& start, const Telemetry::Position& end, float yaw_deg)
{
    const auto mean_latitude_rad =
        (start.latitude_deg + end.latitude_deg) * 0.5 * kDegreesToRadians;
    const auto north_m =
        (end.latitude_deg - start.latitude_deg) * kDegreesToRadians * kEarthRadiusM;
    const auto east_m = (end.longitude_deg - start.longitude_deg) * kDegreesToRadians *
                        kEarthRadiusM * std::cos(mean_latitude_rad);
    const auto yaw_rad = static_cast<double>(yaw_deg) * kDegreesToRadians;
    return {
        north_m * std::cos(yaw_rad) + east_m * std::sin(yaw_rad),
        -north_m * std::sin(yaw_rad) + east_m * std::cos(yaw_rad),
    };
}

bool velocity_is_below(Telemetry& telemetry, float threshold_m_s)
{
    const auto velocity = telemetry.velocity_ned();
    const auto speed = std::hypot(velocity.north_m_s, velocity.east_m_s);
    return speed < threshold_m_s && std::abs(velocity.down_m_s) < threshold_m_s;
}

double signed_yaw_change_deg(float start_deg, float end_deg)
{
    return std::remainder(static_cast<double>(end_deg - start_deg), 360.0);
}

class ArduPilotCompatibility : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        const auto url = ardupilot_url();
        if (!url) {
            return;
        }

        sdk = std::make_unique<Mavsdk>(Mavsdk::Configuration{ComponentType::GroundStation});
        const auto [connection_result, handle] = sdk->add_any_connection_with_handle(*url);
        ASSERT_EQ(connection_result, ConnectionResult::Success);
        connection_handle = handle;
        system = sdk->first_autopilot(kDiscoveryTimeout).value_or(nullptr);
        ASSERT_NE(system, nullptr);
        telemetry = std::make_unique<Telemetry>(system);
        action = std::make_unique<Action>(system);
        offboard = std::make_unique<Offboard>(system);
        passthrough = std::make_unique<MavlinkPassthrough>(system);
        param = std::make_unique<Param>(system);
        geofence = std::make_unique<Geofence>(system);
    }

    static void TearDownTestSuite()
    {
        geofence.reset();
        param.reset();
        passthrough.reset();
        offboard.reset();
        action.reset();
        telemetry.reset();
        system.reset();
        sdk.reset();
    }

    void SetUp() override
    {
        if (!ardupilot_url()) {
            GTEST_SKIP() << "set MAVSDK_ARDUPILOT_URL to run the ArduPilot SITL suite";
        }
        ASSERT_NE(system, nullptr);
    }

    void TearDown() override
    {
        if (!telemetry || !action || !offboard || !telemetry->armed()) {
            return;
        }
        if (offboard->is_active()) {
            offboard->set_velocity_body({});
            offboard->stop();
        }
        action->land();
        EXPECT_TRUE(wait_until([] { return !telemetry->armed(); }, std::chrono::seconds(45)))
            << "vehicle did not land and disarm during test cleanup";
    }

    static std::unique_ptr<Mavsdk> sdk;
    static std::optional<Mavsdk::ConnectionHandle> connection_handle;
    static std::shared_ptr<System> system;
    static std::unique_ptr<Telemetry> telemetry;
    static std::unique_ptr<Action> action;
    static std::unique_ptr<Offboard> offboard;
    static std::unique_ptr<MavlinkPassthrough> passthrough;
    static std::unique_ptr<Param> param;
    static std::unique_ptr<Geofence> geofence;
};

std::unique_ptr<Mavsdk> ArduPilotCompatibility::sdk;
std::optional<Mavsdk::ConnectionHandle> ArduPilotCompatibility::connection_handle;
std::shared_ptr<System> ArduPilotCompatibility::system;
std::unique_ptr<Telemetry> ArduPilotCompatibility::telemetry;
std::unique_ptr<Action> ArduPilotCompatibility::action;
std::unique_ptr<Offboard> ArduPilotCompatibility::offboard;
std::unique_ptr<MavlinkPassthrough> ArduPilotCompatibility::passthrough;
std::unique_ptr<Param> ArduPilotCompatibility::param;
std::unique_ptr<Geofence> ArduPilotCompatibility::geofence;

TEST_F(ArduPilotCompatibility, DiscoversArduPilotFromHeartbeat)
{
    ASSERT_TRUE(system->has_autopilot());
    ASSERT_TRUE(system->is_connected());
    ASSERT_NE(system->get_system_id(), 0);
    EXPECT_EQ(system->autopilot_type(), Autopilot::ArduPilot);

    auto promise = std::make_shared<std::promise<mavlink_heartbeat_t>>();
    auto future = promise->get_future();
    auto delivered = std::make_shared<std::atomic_bool>(false);
    const auto handle = passthrough->subscribe_message(
        MAVLINK_MSG_ID_HEARTBEAT, [promise, delivered](const mavlink_message_t& message) {
            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            if (!delivered->exchange(true)) {
                promise->set_value(heartbeat);
            }
        });

    ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    const auto heartbeat = future.get();
    passthrough->unsubscribe_message(MAVLINK_MSG_ID_HEARTBEAT, handle);
    EXPECT_EQ(heartbeat.autopilot, MAV_AUTOPILOT_ARDUPILOTMEGA);
}

TEST_F(ArduPilotCompatibility, ProvidesCoreTelemetryWithoutManualRateSetup)
{
    const auto position = wait_for_valid_position(*telemetry);
    const auto velocity = wait_for_value<Telemetry::VelocityNed>(
        [](auto callback) { return telemetry->subscribe_velocity_ned(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_velocity_ned(handle); });
    const auto battery = wait_for_value<Telemetry::Battery>(
        [](auto callback) { return telemetry->subscribe_battery(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_battery(handle); });
    const auto gps = wait_for_value<Telemetry::GpsInfo>(
        [](auto callback) { return telemetry->subscribe_gps_info(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_gps_info(handle); });
    const auto attitude = wait_for_value<Telemetry::EulerAngle>(
        [](auto callback) { return telemetry->subscribe_attitude_euler(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_attitude_euler(handle); });

    EXPECT_TRUE(position);
    EXPECT_TRUE(velocity);
    EXPECT_TRUE(battery);
    EXPECT_TRUE(gps);
    EXPECT_TRUE(attitude);
}

TEST_F(ArduPilotCompatibility, ProvidesCoreTelemetryAfterExplicitRateSetup)
{
    ASSERT_EQ(telemetry->set_rate_position(4.0), Telemetry::Result::Success);
    ASSERT_EQ(telemetry->set_rate_velocity_ned(4.0), Telemetry::Result::Success);
    ASSERT_EQ(telemetry->set_rate_battery(2.0), Telemetry::Result::Success);
    ASSERT_EQ(telemetry->set_rate_gps_info(2.0), Telemetry::Result::Success);
    ASSERT_EQ(telemetry->set_rate_attitude_euler(4.0), Telemetry::Result::Success);

    const auto position = wait_for_valid_position(*telemetry);
    const auto velocity = wait_for_value<Telemetry::VelocityNed>(
        [](auto callback) { return telemetry->subscribe_velocity_ned(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_velocity_ned(handle); });
    const auto battery = wait_for_value<Telemetry::Battery>(
        [](auto callback) { return telemetry->subscribe_battery(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_battery(handle); });
    const auto gps = wait_for_value<Telemetry::GpsInfo>(
        [](auto callback) { return telemetry->subscribe_gps_info(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_gps_info(handle); });
    const auto attitude = wait_for_value<Telemetry::EulerAngle>(
        [](auto callback) { return telemetry->subscribe_attitude_euler(std::move(callback)); },
        [](auto handle) { telemetry->unsubscribe_attitude_euler(handle); });

    ASSERT_TRUE(position);
    EXPECT_TRUE(std::isfinite(position->latitude_deg));
    EXPECT_TRUE(std::isfinite(position->longitude_deg));
    EXPECT_TRUE(std::isfinite(position->relative_altitude_m));
    ASSERT_TRUE(velocity);
    EXPECT_TRUE(std::isfinite(velocity->north_m_s));
    EXPECT_TRUE(std::isfinite(velocity->east_m_s));
    EXPECT_TRUE(std::isfinite(velocity->down_m_s));
    ASSERT_TRUE(battery);
    EXPECT_TRUE(std::isfinite(battery->voltage_v));
    ASSERT_TRUE(gps);
    ASSERT_TRUE(attitude);
    EXPECT_TRUE(std::isfinite(attitude->roll_deg));
    EXPECT_TRUE(std::isfinite(attitude->pitch_deg));
    EXPECT_TRUE(std::isfinite(attitude->yaw_deg));
    EXPECT_FALSE(telemetry->armed());
    EXPECT_NE(telemetry->flight_mode(), Telemetry::FlightMode::Unknown);
}

TEST_F(ArduPilotCompatibility, ReadsFloatAndIntegerParameters)
{
    const auto options = OperationOptions{kOperationTimeout};
    const auto [float_result, speed] = param->get_param_float("ATC_RAT_YAW_P", options);
    ASSERT_EQ(float_result, Param::Result::Success);
    EXPECT_GT(speed, 0.0F);

    const auto [int_result, system_id] = param->get_param_int("MAV_SYSID", options);
    ASSERT_EQ(int_result, Param::Result::Success);
    EXPECT_EQ(system_id, system->get_system_id());
}

TEST_F(ArduPilotCompatibility, DecodesArduPilotParameterWireEncoding)
{
    Param wire_param(system);
    struct Capture {
        std::mutex mutex;
        std::optional<mavlink_param_value_t> float_param;
        std::optional<mavlink_param_value_t> integer_param;
    };
    auto capture = std::make_shared<Capture>();
    const auto handle = passthrough->subscribe_message(
        MAVLINK_MSG_ID_PARAM_VALUE, [capture](const mavlink_message_t& message) {
            mavlink_param_value_t value{};
            mavlink_msg_param_value_decode(&message, &value);
            const auto end = std::find(std::begin(value.param_id), std::end(value.param_id), '\0');
            const std::string name(value.param_id, end);
            std::lock_guard lock(capture->mutex);
            if (name == "ATC_RAT_YAW_P") {
                capture->float_param = value;
            } else if (name == "MAV_SYSID") {
                capture->integer_param = value;
            }
        });

    const auto options = OperationOptions{kOperationTimeout};
    const auto [float_result, float_value] = wire_param.get_param_float("ATC_RAT_YAW_P", options);
    const auto [int_result, int_value] = wire_param.get_param_int("MAV_SYSID", options);
    const auto request_wire_parameter = [this](const char* name) {
        const auto target_system = passthrough->get_target_sysid();
        const auto target_component = passthrough->get_target_compid();
        return passthrough->queue_message([=](MavlinkAddress address, uint8_t channel) {
            mavlink_message_t message{};
            mavlink_msg_param_request_read_pack_chan(
                address.system_id,
                address.component_id,
                channel,
                &message,
                target_system,
                target_component,
                name,
                -1);
            return message;
        });
    };
    ASSERT_EQ(request_wire_parameter("ATC_RAT_YAW_P"), MavlinkPassthrough::Result::Success);
    ASSERT_EQ(request_wire_parameter("MAV_SYSID"), MavlinkPassthrough::Result::Success);
    ASSERT_TRUE(wait_until(
        [capture] {
            std::lock_guard lock(capture->mutex);
            return capture->float_param.has_value() && capture->integer_param.has_value();
        },
        std::chrono::seconds(2)));
    passthrough->unsubscribe_message(MAVLINK_MSG_ID_PARAM_VALUE, handle);

    ASSERT_EQ(float_result, Param::Result::Success);
    ASSERT_EQ(int_result, Param::Result::Success);
    std::lock_guard lock(capture->mutex);
    ASSERT_TRUE(capture->float_param);
    ASSERT_TRUE(capture->integer_param);
    EXPECT_EQ(capture->float_param->param_type, MAV_PARAM_TYPE_REAL32);
    EXPECT_FLOAT_EQ(capture->float_param->param_value, float_value);
    EXPECT_NE(capture->integer_param->param_type, MAV_PARAM_TYPE_REAL32);
    EXPECT_FLOAT_EQ(capture->integer_param->param_value, static_cast<float>(int_value));
}

TEST_F(ArduPilotCompatibility, ReportsMissingParameter)
{
    const auto started = std::chrono::steady_clock::now();
    const auto [result, value] =
        param->get_param_float("MAVSDK_MISSING", OperationOptions{std::chrono::seconds(2)});
    const auto elapsed = std::chrono::steady_clock::now() - started;
    (void)value;
    EXPECT_EQ(result, Param::Result::DoesNotExist);
    EXPECT_LT(elapsed, std::chrono::seconds(3));
}

MavlinkPassthrough::CommandLong
make_command_long(MavlinkPassthrough& passthrough, std::uint16_t command)
{
    MavlinkPassthrough::CommandLong value{};
    value.target_sysid = passthrough.get_target_sysid();
    value.target_compid = passthrough.get_target_compid();
    value.command = command;
    return value;
}

TEST_F(ArduPilotCompatibility, MapsAcceptedAndUnsupportedCommandAcks)
{
    auto request = make_command_long(*passthrough, MAV_CMD_REQUEST_MESSAGE);
    request.param1 = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
    EXPECT_EQ(passthrough->send_command_long(request), MavlinkPassthrough::Result::Success);

    const auto unsupported = make_command_long(*passthrough, 65534);
    EXPECT_EQ(
        passthrough->send_command_long(unsupported),
        MavlinkPassthrough::Result::CommandUnsupported);
}

TEST_F(ArduPilotCompatibility, MapsRejectedCommandAck)
{
    MavlinkPassthrough::CommandInt command{};
    command.target_sysid = passthrough->get_target_sysid();
    command.target_compid = passthrough->get_target_compid();
    command.command = MAV_CMD_DO_REPOSITION;
    command.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    command.param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    command.x = 910000000;
    command.y = 0;
    command.z = 10.0F;

    EXPECT_EQ(passthrough->send_command_int(command), MavlinkPassthrough::Result::CommandDenied);
}

TEST_F(ArduPilotCompatibility, TimesOutWhenCommandTargetIsSilent)
{
    auto command = make_command_long(*passthrough, MAV_CMD_REQUEST_MESSAGE);
    command.target_compid = 250;
    command.param1 = MAVLINK_MSG_ID_AUTOPILOT_VERSION;

    const auto started = std::chrono::steady_clock::now();
    EXPECT_EQ(
        passthrough->send_command_long(command, OperationOptions{kOperationTimeout}),
        MavlinkPassthrough::Result::CommandTimeout);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_GE(elapsed, std::chrono::milliseconds(200));
    EXPECT_LT(elapsed, std::chrono::seconds(1));
}

TEST_F(ArduPilotCompatibility, TimesOutCommandWhileWaitingBehindSilentCommand)
{
    auto silent = make_command_long(*passthrough, MAV_CMD_REQUEST_MESSAGE);
    silent.target_compid = 250;
    silent.param1 = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
    auto first = std::async(std::launch::async, [silent] {
        return passthrough->send_command_long(
            silent, OperationOptions{std::chrono::milliseconds(1500)});
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto queued = make_command_long(*passthrough, MAV_CMD_REQUEST_MESSAGE);
    queued.param1 = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
    const auto started = std::chrono::steady_clock::now();
    const auto result = passthrough->send_command_long(queued, OperationOptions{kOperationTimeout});
    const auto elapsed = std::chrono::steady_clock::now() - started;

    EXPECT_EQ(result, MavlinkPassthrough::Result::CommandTimeout);
    EXPECT_LT(elapsed, std::chrono::milliseconds(900));
    EXPECT_EQ(first.get(), MavlinkPassthrough::Result::CommandTimeout);
}

TEST_F(ArduPilotCompatibility, TimesOutParameterWhileWaitingBehindSilentParameter)
{
    Param silent_param(system);
    (void)silent_param.select_component(1, Param::ProtocolVersion::Ext);
    auto first = std::async(std::launch::async, [&silent_param] {
        return silent_param.get_param_float(
            "MAV_SYSID", OperationOptions{std::chrono::milliseconds(1500)});
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto started = std::chrono::steady_clock::now();
    const auto [result, value] =
        silent_param.get_param_int("MAV_SYSID", OperationOptions{kOperationTimeout});
    const auto elapsed = std::chrono::steady_clock::now() - started;
    (void)value;

    EXPECT_EQ(result, Param::Result::Timeout);
    EXPECT_LT(elapsed, std::chrono::milliseconds(900));
    EXPECT_EQ(first.get().first, Param::Result::Timeout);
}

TEST_F(ArduPilotCompatibility, AcceptsRelativeAltitudeRepositionAsCommandInt)
{
    ASSERT_EQ(telemetry->set_rate_position(2.0), Telemetry::Result::Success);
    ASSERT_TRUE(wait_for_guided_position_ready(*telemetry));
    const auto position = wait_for_valid_position(*telemetry);
    ASSERT_TRUE(position);

    MavlinkPassthrough::CommandInt command{};
    command.target_sysid = passthrough->get_target_sysid();
    command.target_compid = passthrough->get_target_compid();
    command.command = MAV_CMD_DO_REPOSITION;
    command.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    command.param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    command.param4 = NAN;
    command.x = static_cast<std::int32_t>(std::llround(position->latitude_deg * 1e7));
    command.y = static_cast<std::int32_t>(std::llround(position->longitude_deg * 1e7));
    command.z = 10.0F;

    EXPECT_EQ(passthrough->send_command_int(command), MavlinkPassthrough::Result::Success);
}

TEST_F(ArduPilotCompatibility, ActionRepositionsUsingRelativeAltitude)
{
    ASSERT_EQ(telemetry->set_rate_position(2.0), Telemetry::Result::Success);
    ASSERT_TRUE(wait_for_armable_position(*telemetry));
    const auto position = wait_for_valid_position(*telemetry);
    ASSERT_TRUE(position);

    EXPECT_EQ(
        action->goto_location_relative(
            position->latitude_deg,
            position->longitude_deg,
            10.0F,
            NAN,
            OperationOptions{std::chrono::milliseconds::zero()}),
        Action::Result::Timeout);

    ASSERT_EQ(action->set_takeoff_altitude(4.0F), Action::Result::Success);
    ASSERT_EQ(action->arm(), Action::Result::Success);
    ASSERT_EQ(action->takeoff(), Action::Result::Success);
    ASSERT_TRUE(wait_until(
        [] { return telemetry->in_air() && telemetry->position().relative_altitude_m > 3.0F; },
        std::chrono::seconds(30)));

    auto target = telemetry->position();
    target.latitude_deg += 5.0 / (kEarthRadiusM * kDegreesToRadians);
    constexpr float target_relative_altitude_m = 6.0F;
    ASSERT_EQ(
        action->goto_location_relative(
            target.latitude_deg,
            target.longitude_deg,
            target_relative_altitude_m,
            NAN,
            OperationOptions{std::chrono::seconds(5)}),
        Action::Result::Success);
    EXPECT_TRUE(wait_until(
        [target, target_relative_altitude_m] {
            const auto actual = telemetry->position();
            const auto displacement = body_displacement(target, actual, 0.0F);
            const auto horizontal_error = std::hypot(displacement.forward_m, displacement.right_m);
            return horizontal_error < 1.5 &&
                   std::abs(actual.relative_altitude_m - target_relative_altitude_m) < 0.8F;
        },
        std::chrono::seconds(30)))
        << "reposition was ACKed but did not reach the relative-altitude target";

    ASSERT_EQ(action->land(), Action::Result::Success);
    ASSERT_TRUE(wait_until([] { return !telemetry->armed(); }, std::chrono::seconds(45)));
}

TEST_F(ArduPilotCompatibility, ExecutesGuidedBodyVelocityAndStops)
{
    ASSERT_EQ(telemetry->set_rate_position(5.0), Telemetry::Result::Success);
    ASSERT_EQ(telemetry->set_rate_velocity_ned(10.0), Telemetry::Result::Success);
    ASSERT_EQ(telemetry->set_rate_attitude_euler(10.0), Telemetry::Result::Success);
    ASSERT_TRUE(wait_for_armable_position(*telemetry))
        << "ArduCopter did not become armable with a global position estimate";

    ASSERT_EQ(action->set_takeoff_altitude(5.0F), Action::Result::Success);
    ASSERT_EQ(action->arm(), Action::Result::Success);
    ASSERT_EQ(action->takeoff(), Action::Result::Success);
    ASSERT_TRUE(wait_until(
        [] { return telemetry->in_air() && telemetry->position().relative_altitude_m > 3.5F; },
        std::chrono::seconds(30)));

    ASSERT_EQ(offboard->set_velocity_body({}), Offboard::Result::Success);
    ASSERT_EQ(offboard->start(), Offboard::Result::Success);
    ASSERT_TRUE(offboard->is_active());
    ASSERT_TRUE(
        wait_until([] { return velocity_is_below(*telemetry, 0.25F); }, std::chrono::seconds(5)));

    const auto forward_start = telemetry->position();
    const auto forward_yaw = telemetry->attitude_euler().yaw_deg;
    ASSERT_EQ(offboard->set_velocity_body({0.75F, 0.0F, 0.0F, 0.0F}), Offboard::Result::Success);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    const auto forward_mid = telemetry->position();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    const auto forward_end = telemetry->position();
    const auto first_forward = body_displacement(forward_start, forward_mid, forward_yaw);
    const auto repeated_forward = body_displacement(forward_mid, forward_end, forward_yaw);
    EXPECT_GT(first_forward.forward_m, 1.0);
    EXPECT_LT(std::abs(first_forward.right_m), 1.5);
    EXPECT_GT(repeated_forward.forward_m, 0.4)
        << "one set_velocity_body call must keep producing repeated setpoints";

    ASSERT_EQ(offboard->set_velocity_body({}), Offboard::Result::Success);
    ASSERT_TRUE(
        wait_until([] { return velocity_is_below(*telemetry, 0.25F); }, std::chrono::seconds(5)));
    const auto right_start = telemetry->position();
    const auto right_yaw = telemetry->attitude_euler().yaw_deg;
    ASSERT_EQ(offboard->set_velocity_body({0.0F, 0.75F, 0.0F, 0.0F}), Offboard::Result::Success);
    ASSERT_TRUE(wait_until(
        [right_start, right_yaw] {
            const auto moved = body_displacement(right_start, telemetry->position(), right_yaw);
            return moved.right_m > 1.0 && std::abs(moved.forward_m) < 1.5;
        },
        std::chrono::seconds(8)));

    ASSERT_EQ(offboard->set_velocity_body({}), Offboard::Result::Success);
    ASSERT_TRUE(
        wait_until([] { return velocity_is_below(*telemetry, 0.25F); }, std::chrono::seconds(5)));
    const auto altitude_start = telemetry->position().relative_altitude_m;
    ASSERT_EQ(offboard->set_velocity_body({0.0F, 0.0F, 0.35F, 0.0F}), Offboard::Result::Success);
    ASSERT_TRUE(wait_until(
        [altitude_start] {
            return telemetry->position().relative_altitude_m < altitude_start - 0.4F;
        },
        std::chrono::seconds(8)));

    ASSERT_EQ(offboard->set_velocity_body({}), Offboard::Result::Success);
    ASSERT_TRUE(
        wait_until([] { return velocity_is_below(*telemetry, 0.25F); }, std::chrono::seconds(5)));
    const auto yaw_start = telemetry->attitude_euler().yaw_deg;
    ASSERT_EQ(offboard->set_velocity_body({0.0F, 0.0F, 0.0F, 20.0F}), Offboard::Result::Success);
    ASSERT_TRUE(wait_until(
        [yaw_start] {
            const auto yaw_now = telemetry->attitude_euler().yaw_deg;
            return signed_yaw_change_deg(yaw_start, yaw_now) > 8.0;
        },
        std::chrono::seconds(5)));

    ASSERT_EQ(offboard->set_velocity_body({}), Offboard::Result::Success);
    ASSERT_TRUE(
        wait_until([] { return velocity_is_below(*telemetry, 0.25F); }, std::chrono::seconds(5)));
    ASSERT_EQ(offboard->stop(), Offboard::Result::Success);
    EXPECT_FALSE(offboard->is_active());

    ASSERT_EQ(action->land(), Action::Result::Success);
    ASSERT_TRUE(wait_until([] { return !telemetry->armed(); }, std::chrono::seconds(45)));
}

TEST_F(ArduPilotCompatibility, OneShotVelocityExpiresWhenProducerStalls)
{
    ASSERT_TRUE(wait_for_armable_position(*telemetry));
    const auto [timeout_result, guided_timeout] = param->get_param_float("GUID_TIMEOUT");
    ASSERT_EQ(timeout_result, Param::Result::Success);
    ASSERT_GT(guided_timeout, 0.0F);
    ASSERT_LE(guided_timeout, 10.0F);
    ASSERT_EQ(telemetry->set_rate_velocity_ned(10.0), Telemetry::Result::Success);
    ASSERT_EQ(action->set_takeoff_altitude(5.0F), Action::Result::Success);
    ASSERT_EQ(action->arm(), Action::Result::Success);
    ASSERT_EQ(action->takeoff(), Action::Result::Success);
    ASSERT_TRUE(wait_until(
        [] { return telemetry->in_air() && telemetry->position().relative_altitude_m > 3.5F; },
        std::chrono::seconds(30)));
    ASSERT_EQ(
        offboard->set_velocity_body_once({0.75F, 0.0F, 0.0F, 0.0F}), Offboard::Result::Success);
    ASSERT_TRUE(wait_until(
        [] {
            const auto velocity = telemetry->velocity_ned();
            return std::hypot(velocity.north_m_s, velocity.east_m_s) > 0.3F;
        },
        std::chrono::seconds(2)));
    // No producer refresh and no application zero: ArduPilot must expire the command.
    std::this_thread::sleep_for(std::chrono::duration<float>(guided_timeout));
    EXPECT_TRUE(
        wait_until([] { return velocity_is_below(*telemetry, 0.25F); }, std::chrono::seconds(5)));
    EXPECT_FALSE(offboard->is_active());
    ASSERT_EQ(offboard->set_velocity_body_once({}), Offboard::Result::Success);
    ASSERT_EQ(action->land(), Action::Result::Success);
    ASSERT_TRUE(wait_until([] { return !telemetry->armed(); }, std::chrono::seconds(45)));
}

TEST_F(ArduPilotCompatibility, UploadsAndDownloadsInclusionFence)
{
    ASSERT_EQ(telemetry->set_rate_position(2.0), Telemetry::Result::Success);
    const auto position = wait_for_valid_position(*telemetry);
    ASSERT_TRUE(position);

    Geofence::Polygon polygon{};
    polygon.fence_type = Geofence::FenceType::Inclusion;
    polygon.points = {
        {position->latitude_deg + 0.001, position->longitude_deg - 0.001},
        {position->latitude_deg + 0.001, position->longitude_deg + 0.001},
        {position->latitude_deg - 0.001, position->longitude_deg + 0.001},
        {position->latitude_deg - 0.001, position->longitude_deg - 0.001},
    };
    Geofence::GeofenceData upload{};
    upload.polygons.push_back(polygon);

    ASSERT_EQ(geofence->upload_geofence(upload), Geofence::Result::Success);
    const auto [result, download] = geofence->download_geofence();
    ASSERT_EQ(result, Geofence::Result::Success);
    ASSERT_EQ(download.polygons.size(), 1);
    EXPECT_EQ(download.polygons.front().fence_type, Geofence::FenceType::Inclusion);
    ASSERT_EQ(download.polygons.front().points.size(), polygon.points.size());
    for (std::size_t index = 0; index < polygon.points.size(); ++index) {
        EXPECT_NEAR(
            download.polygons.front().points[index].latitude_deg,
            polygon.points[index].latitude_deg,
            1e-5);
        EXPECT_NEAR(
            download.polygons.front().points[index].longitude_deg,
            polygon.points[index].longitude_deg,
            1e-5);
    }

    EXPECT_EQ(geofence->clear_geofence(), Geofence::Result::Success);
}

TEST_F(ArduPilotCompatibility, RejectsInvalidFencePolygon)
{
    ASSERT_EQ(telemetry->set_rate_position(2.0), Telemetry::Result::Success);
    const auto position = wait_for_valid_position(*telemetry);
    ASSERT_TRUE(position);

    Geofence::Polygon polygon{};
    polygon.fence_type = Geofence::FenceType::Inclusion;
    polygon.points = {
        {position->latitude_deg, position->longitude_deg},
        {position->latitude_deg + 0.001, position->longitude_deg + 0.001},
    };
    Geofence::GeofenceData invalid{};
    invalid.polygons.push_back(polygon);

    EXPECT_EQ(geofence->upload_geofence(invalid), Geofence::Result::InvalidArgument);
    EXPECT_EQ(geofence->clear_geofence(), Geofence::Result::Success);
}

TEST_F(ArduPilotCompatibility, ReconnectsAfterConnectionInterruption)
{
    ASSERT_TRUE(connection_handle);
    sdk->remove_connection(*connection_handle);
    connection_handle.reset();
    ASSERT_TRUE(wait_until([] { return !system->is_connected(); }, std::chrono::seconds(10)));

    const auto url = ardupilot_url();
    ASSERT_TRUE(url);
    const auto [connection_result, handle] = sdk->add_any_connection_with_handle(*url);
    ASSERT_EQ(connection_result, ConnectionResult::Success);
    connection_handle = handle;

    ASSERT_TRUE(wait_until([] { return system->is_connected(); }, std::chrono::seconds(20)));
    EXPECT_EQ(system->autopilot_type(), Autopilot::ArduPilot);
}

} // namespace

# ArduPilot compatibility fork

The `YoussGm3o8/MAVSDK` `nomad/ardupilot` branch is an independently buildable
MAVSDK fork. Its production code may contain only functionality that is useful
to any MAVSDK application communicating with ArduPilot.

The fork owns protocol representation, command and parameter transport,
flight-stack compatibility, message-rate behavior, mission and geofence
interoperability, and ArduPilot SITL regression tests. Applications remain
responsible for vehicle selection, authorization, operating limits, freshness
policy, watchdogs, failsafes, command sequencing, and authoritative
post-command state verification.

## Upstream tracking

Keep fork changes as a short, reviewable series above upstream MAVSDK:

1. Fetch `mavlink/MAVSDK` into the `upstream` remote.
2. Rebase `nomad/ardupilot` onto the reviewed upstream commit.
3. Resolve and test one generic compatibility patch at a time.
4. Run the normal MAVSDK unit/system tests and the ArduPilot SITL suite.
5. Record the new upstream and ArduPilot pins in this document.
6. Update consumers only after the fork workflow is green.

Do not merge consumer code into this branch to make a rebase pass. A patch that
cannot be explained without referring to one application's policy does not
belong in this fork.

The existing Linux, macOS, and Windows workflows also run for pushes to
`nomad/ardupilot`. The dedicated ArduPilot workflow supplements those checks;
it does not replace normal MAVSDK qualification.

The local Windows qualification on 2026-09-18 ran the focused normal MAVSDK
system-test groups (`Param`, `Action`, `Geofence`, `Connections`, `Telemetry`,
`HeartbeatWatchdog`, and `FirstHeartbeat`) and passed all 28 tests. This includes
UDP reconnection after the peer temporarily disappears; Windows can report the
peer's ICMP port-unreachable response as a transient receive error, so the UDP
receive loop retries while its socket remains open.

## Reproducible test inputs

| Input | Pin |
|---|---|
| Upstream MAVSDK base | `34b417d45c2c33ce0414bc1bc61b54010d055224` |
| ArduCopter release | `Copter-4.7.1` |
| ArduPilot commit | `dbe792162d06cab66c3475fd5556bf7a120f119e` |
| SITL Docker build context | `radarku/ardupilot-sitl-docker@eff32c1f98152ac3d1dc09a1e475733b73ce569f` |

The workflow starts the ArduCopter binary directly instead of routing it
through MAVProxy. This prevents another GCS from configuring stream rates before
MAVSDK connects. ArduCopter sends MAVLink directly to MAVSDK's UDP listener.

## Running the suite

On Linux, build the MAVSDK runner and the pinned SITL image, then connect them
directly over UDP:

```sh
cmake -S cpp -B build -DBUILD_TESTING=ON -DBUILD_MAVSDK_SERVER=OFF
cmake --build build --target ardupilot_tests_runner -j2
docker build \
  --build-arg COPTER_TAG=dbe792162d06cab66c3475fd5556bf7a120f119e \
  --tag mavsdk-ardupilot-sitl:copter-4.7.1 \
  https://github.com/radarku/ardupilot-sitl-docker.git#eff32c1f98152ac3d1dc09a1e475733b73ce569f
docker run --detach --rm --network host --name mavsdk-ardupilot-sitl \
  --entrypoint /ardupilot/build/sitl/bin/arducopter \
  mavsdk-ardupilot-sitl:copter-4.7.1 \
  -w --model quad --speedup 1 --slave 0 \
  --defaults Tools/autotest/default_params/copter.parm \
  --serial0 udpclient:127.0.0.1:14540 \
  --sim-address=127.0.0.1 -I0 \
  --home 42.3898,-71.1476,14.0,270.0
MAVSDK_ARDUPILOT_URL=udpin://0.0.0.0:14540 \
  build/src/system_tests/ardupilot_tests_runner \
  --gtest_filter='ArduPilotCompatibility.*:ActionTransport.*:GeofenceTransport.*'
docker rm --force mavsdk-ardupilot-sitl
```

On Windows with Docker Desktop, publish SITL's TCP telemetry port instead of
using the container-to-host UDP route. The same pinned image and test binary
are used; restart the container for a clean-session run:

```powershell
docker run --detach --rm --publish 5762:5762/tcp --name mavsdk-ardupilot-sitl `
  --entrypoint /ardupilot/build/sitl/bin/arducopter mavsdk-ardupilot-sitl:copter-4.7.1 `
  -w --model quad --speedup 1 --slave 0 `
  --defaults Tools/autotest/default_params/copter.parm `
  --serial0 udpclient:127.0.0.1:14540 --sim-address=127.0.0.1 -I0 `
  --home 42.3898,-71.1476,14.0,270.0
$env:MAVSDK_ARDUPILOT_URL = 'tcpout://127.0.0.1:5762'
& .\build\src\system_tests\Debug\ardupilot_tests_runner.exe `
  --gtest_filter='ArduPilotCompatibility.*:ActionTransport.*:GeofenceTransport.*'
docker stop mavsdk-ardupilot-sitl
```

The Windows route passed all 19 tests on 2026-09-18. The dedicated Linux CI
workflow retains direct UDP with no other GCS connected.

The tests skip when `MAVSDK_ARDUPILOT_URL` is absent, so the ordinary hermetic
system-test job does not accidentally depend on a simulator. The dedicated
workflow treats a missing or failed simulator as an error and always removes
the container.

## Compatibility matrix

`Tested` means the named check exists and passed against the pinned inputs. The
dedicated workflow enforces the same topology on branch pushes and pull
requests. `Unknown` means no such evidence is claimed.

| Feature | ArduCopter | Test | Status |
|---|---|---|---|
| UDP connection and discovery | 4.7.1 | `DiscoversArduPilotFromHeartbeat` | Tested |
| Outbound GCS heartbeat after discovery | synthetic autopilot | `ActionTransport.SendsRelativeRepositionAsCommandInt` | Tested |
| Clean-session telemetry initialization | 4.7.1 | `ProvidesCoreTelemetryWithoutManualRateSetup` | Tested |
| Position and relative-altitude telemetry | 4.7.1 | `ProvidesCoreTelemetryAfterExplicitRateSetup` | Tested |
| Velocity NED telemetry | 4.7.1 | `ProvidesCoreTelemetryAfterExplicitRateSetup` | Tested |
| Battery telemetry | 4.7.1 | `ProvidesCoreTelemetryAfterExplicitRateSetup` | Tested |
| GPS information and Euler attitude | 4.7.1 | `ProvidesCoreTelemetryAfterExplicitRateSetup` | Tested |
| Flight mode and armed state | 4.7.1 | `ProvidesCoreTelemetryAfterExplicitRateSetup` | Tested |
| Float parameter read | 4.7.1 | `ReadsFloatAndIntegerParameters` | Tested |
| Integer parameter read | 4.7.1 | `ReadsFloatAndIntegerParameters` | Tested |
| Parameter wire encoding | 4.7.1 | `DecodesArduPilotParameterWireEncoding` | Tested |
| Missing parameter | 4.7.1 | `ReportsMissingParameter` | Tested |
| `COMMAND_LONG` ACK mapping | 4.7.1 | `MapsAcceptedAndUnsupportedCommandAcks` | Tested |
| Rejected command mapping | 4.7.1 | `MapsRejectedCommandAck` | Tested |
| Silent command target timeout | 4.7.1 | `TimesOutWhenCommandTargetIsSilent` | Tested |
| `COMMAND_INT` relative-altitude reposition | 4.7.1 | `AcceptsRelativeAltitudeRepositionAsCommandInt` | Tested |
| High-level goto wire frame and coordinate capture | synthetic autopilot | `ActionTransport.SendsRelativeRepositionAsCommandInt` | Tested |
| High-level relative-altitude goto ACK | 4.7.1 | `ActionRepositionsUsingRelativeAltitude` | Tested |
| High-level relative-altitude movement | 4.7.1 | `ActionRepositionsUsingRelativeAltitude` | Tested |
| Inclusion-fence upload/download | 4.7.1 | `UploadsAndDownloadsInclusionFence` | Tested |
| Disconnect and reconnect | 4.7.1 | `ReconnectsAfterConnectionInterruption` | Tested |
| Operation-scoped command/parameter deadline | 4.7.1 | `OperationTimeout` unit + SITL queued/silent tests | Tested |
| Guided body velocity and stop | 4.7.1 | `ExecutesGuidedBodyVelocityAndStops` | Tested |
| Invalid geofence input rejection | 4.7.1 | `RejectsInvalidFencePolygon` | Tested |
| Vehicle-side rejected fence upload | 4.7.1 | — | Unknown |
| Geofence download timeout for a silent peer | synthetic autopilot | `GeofenceTransport.TimesOutWhenAutopilotIgnoresFenceDownload` | Tested |
| Geofence timeout against ArduCopter | 4.7.1 | — | Unknown |
| Calibration | — | — | Unknown |
| Mission high-level API | — | — | Unknown |

On ArduPilot, the Telemetry plugin requests its core message set with
`MAV_CMD_SET_MESSAGE_INTERVAL` when the plugin is enabled. The clean-session
test subscribes without calling `Telemetry::set_rate_*` and without another GCS
connected. A legacy `REQUEST_DATA_STREAM` fallback remains unknown until a
pinned ArduPilot version that requires it is identified and tested.

## Current API findings

- `Action::goto_location_relative` sends `MAV_CMD_DO_REPOSITION` as
  `COMMAND_INT` with `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`. Its
  `OperationOptions` overload applies one deadline to the optional Guided-mode
  change and the reposition command. The pinned-SITL test takes off, commands
  a displaced point and new relative altitude, verifies arrival, then lands.
  A synthetic-autopilot test independently captures the encoded command frame,
  coordinates, and relative altitude.
- `Offboard::set_velocity_body` already represents forward, right, down, and
  yaw-rate setpoints and repeats them. It sends `MAV_FRAME_BODY_NED`; for
  velocity and acceleration values the MAVLink specification defines that as
  the body FRD frame, which is also the meaning of `MAV_FRAME_BODY_OFFSET_NED`.
  Consumers whose existing contract stores yaw rate in radians per second must
  convert it because `VelocityBodyYawspeed::yawspeed_deg_s` is degrees per
  second. The pinned-SITL test now covers Guided forward, right, down, yaw-rate,
  repeated updates, zero velocity, and stop behavior. `Offboard` repeats the last
  setpoint automatically; applications that require each setpoint to expire
  unless their own control loop sends a fresh one must account for that semantic
  difference before replacing a one-shot transport.
- `OperationOptions::timeout` bounds the complete command or parameter read,
  including time spent queued and all internal retries. Existing overloads retain
  the SDK-wide per-attempt timeout for backward compatibility.

## Upstream references

- [ArduPilot support tracker](https://github.com/mavlink/MAVSDK/issues/1568)
- [Original ArduPilot architecture discussion](https://github.com/mavlink/MAVSDK/issues/728)
- [ArduPilot/PX4 compatibility report](https://github.com/mavlink/MAVSDK/issues/1996)
- [Telemetry-rate compatibility work](https://github.com/mavlink/MAVSDK/issues/2516)

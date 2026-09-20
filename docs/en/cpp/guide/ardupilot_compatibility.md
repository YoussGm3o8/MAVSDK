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
| Upstream MAVSDK base | `d7043d3cafe8cd6250565fd211b966d8b455d561` |
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
  build/src/system_tests/ardupilot_tests_runner
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
& .\build\src\system_tests\Debug\ardupilot_tests_runner.exe
docker stop mavsdk-ardupilot-sitl
```

The Windows route passed all 19 tests on 2026-09-18. The dedicated Linux CI
workflow retains direct UDP with no other GCS connected.

On 2026-09-19, a fresh Windows TCP session against the same pinned image passed
all 33 tests after the upstream rebase and fault-path additions. This includes
the five one-shot wire tests, four deadline wire tests, four vehicle fence
fault/round-trip tests, and the one-shot producer-stall flight test. Hosted
platform qualification remains a separate requirement for each consumer pin.

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
| Vehicle-side rejected fence upload and recovery | 4.7.1 | `ArduPilotFenceFaults.ReportsVehicleRejectedUploadAndRecovers` | Tested |
| Exclusion-fence polygon round trip | 4.7.1 | `ArduPilotFenceFaults.RoundTripsExclusionPolygon` | Tested |
| Geofence download timeout for a silent peer | synthetic autopilot | `GeofenceTransport.TimesOutWhenAutopilotIgnoresFenceDownload` | Tested |
| Fence upload response loss, timeout and recovery | 4.7.1 | `ArduPilotFenceFaults.UploadTimesOutDuringResponseLossAndRecovers` | Tested |
| Fence download response loss, timeout and recovery | 4.7.1 | `ArduPilotFenceFaults.DownloadTimesOutDuringResponseLossAndRecovers` | Tested |
| One-shot velocity expires with a stalled producer | 4.7.1 | `OneShotVelocityExpiresWhenProducerStalls` | Tested |
| One-shot frame, invalid input, zero, shutdown, lost connection and resend exclusion | synthetic autopilot | `OffboardOneShot.*` (five tests) | Tested |
| Command retry after controlled loss | synthetic autopilot | `OperationDeadlineWire.RetriesDroppedCommandThenSucceedsWithinBudget` | Tested |
| Parameter retry after controlled loss | synthetic autopilot | `OperationDeadlineWire.RetriesDroppedParameterReadThenReturnsValue` | Tested |
| Repeated progress cannot extend a deadline | synthetic autopilot | `OperationDeadlineWire.ProgressAcknowledgementsCannotExtendDeadline` | Tested |
| Concurrent in-flight commands keep independent deadlines | synthetic autopilot | `OperationDeadlineWire.ConcurrentCommandsKeepIndependentDeadlines` | Tested |
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
  repeated updates, zero velocity, and stop behavior.
- `Offboard::set_velocity_body_once` uses the same encoding without storing a
  setpoint or starting a resend timer. It does not change flight mode. It rejects
  nonfinite values, a missing transport connection, and mixing with automatic
  Offboard resends. Success means queued for transport, not vehicle acceptance.
  The application still owns freshness, watchdogs, cancellation, final zeros,
  authorization and authoritative outcome checks. The Copter regression proves
  horizontal movement followed by expiry under its unchanged `GUID_TIMEOUT`
  when the producer stops sending. The wire tests independently prove no resend
  after a stalled producer, a zero followed by destruction, or a removed link.
- `OperationOptions::timeout` bounds the complete command or parameter read,
  including time spent queued and all internal retries. Existing overloads retain
  the SDK-wide per-attempt timeout for backward compatibility.
  Operation intervals round upward to the timer's millisecond precision so a
  progress ACK cannot produce a spurious retry immediately before the deadline.
  `OperationTimeout.FinalAttemptNeverRoundsBeforeTheDeadline` covers that boundary.

The fence rejection test uploads a valid MAVLink plan larger than the pinned
vehicle's fence storage and independently observes `MAV_MISSION_NO_SPACE`.
Timeout tests drop actual vehicle responses, count those dropped replies, bound
completion, restore the response path, and verify a fresh upload/download.
They do not replace application fence policy or assert flight containment.

## Hosted build compatibility

The fork retains Debian 11 packaging despite upstream removing that job.
Bullseye's live security index references OpenSSL and ICU packages that return
404. The job uses the dated `20260830T000000Z` Debian and Debian Security
snapshots, with signature verification retained and expiry checking disabled
only for those dated sources. Clean-container package installation is tested;
this frozen build environment does not claim continuing Debian security support.

All platforms, including iOS device and simulator builds, apply the same MAVLink
patch once. It uses the pinned nested generator without a build-time pip install.
The removed iOS patch attempted to remove the same pip code a second time.

Windows ZIP files are created under `cpp/`. Upload and release actions use those
repository-root-relative paths, and a missing combined archive fails CI.

## Upstream references

- [Pinned generator and shared platform patch contribution](https://github.com/mavlink/MAVSDK/pull/3102)
- [Geofence input validation contribution](https://github.com/mavlink/MAVSDK/pull/3103)
- [Windows archive upload path contribution](https://github.com/mavlink/MAVSDK/pull/3104)

- [ArduPilot support tracker](https://github.com/mavlink/MAVSDK/issues/1568)
- [Original ArduPilot architecture discussion](https://github.com/mavlink/MAVSDK/issues/728)
- [ArduPilot/PX4 compatibility report](https://github.com/mavlink/MAVSDK/issues/1996)
- [Telemetry-rate compatibility work](https://github.com/mavlink/MAVSDK/issues/2516)

# Fixed Map Localization Validation

## Current Asset

- Map: `parking_lot_2026-08-10_v2`.
- PCD: `/var/lib/mower/maps/parking_lot_2026-08-10_v2/map.pcd`.
- SHA-256: `76ea96b69cbdf5862f06ed51a7a37902f12b0a239442907d8c5a48f0bdd13c3d`.
- Raw PCD points: `717170`.
- ICP map points after `0.30 m` voxel filtering: `196865`.

The PCD coordinate system is the FAST-LIO `map_capture_odom` origin from the
mapping run. For localization, this coordinate system is called `map`; a
candidate alignment is therefore `map -> odom`.

## Owner Rule

`fixed_map_localizer` is the only candidate owner of `map -> odom`. It starts
with `publish_tf:=false` and publishes only:

```text
/fixed_map_localization/odometry
```

Do not start the RTK global EKF as a TF owner. Do not enable Nav2 or vehicle
command nodes during this validation.

## Offline Replay Check

Use the captured mapping bag to confirm the map can be loaded and ICP accepts
the original trajectory. This is an observation-only check.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/MobileEss_ws/install/setup.bash

ros2 launch fast_lio fixed_map_localization.launch.py \
  odom_frame:=map_capture_odom \
  publish_tf:=false \
  initial_x_m:=0.0 initial_y_m:=0.0 initial_yaw_rad:=0.0
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/MobileEss_ws/install/setup.bash

ros2 bag play ~/rosbags/parking_lot_2026-08-10_v2/mapping_capture \
  --topics /cloud_registered
```

Terminal 3:

```bash
ros2 topic echo /fixed_map_localization/odometry
```

Expected result: messages arrive near `1 Hz`; covariance `[0]`, `[7]`, and
`[35]` contain a finite ICP fitness score no greater than `0.10`. Rejected ICP
updates are logged and do not publish an estimate.

## Live Observation Check

After replay accepts the source trajectory, use the same physical start area
and vehicle heading as the map capture. Only the Livox driver, real FAST-LIO,
local EKF, static sensor transforms, and fixed-map localizer may run.

1. Start the real FAST-LIO path with `publish_tf:=false` and its odometry frame
   set to `odom`.
2. Start local EKF so it owns `odom -> base_link`.
3. Start the localizer with `publish_tf:=false`:

```bash
ros2 launch fast_lio fixed_map_localization.launch.py \
  publish_tf:=false \
  initial_x_m:=0.0 initial_y_m:=0.0 initial_yaw_rad:=0.0
```

4. Keep the vehicle stationary and compare the candidate against the fixed map
   in RViz. If ICP rejects repeatedly, stop and adjust only the surveyed/manual
   initial estimate. Do not enable TF.
5. Drive under manual control at low speed through the mapped start area and a
   separate route. Record `/cloud_registered`, both odometry topics, `/tf`,
   `/tf_static`, RTK topics, and `/fixed_map_localization/odometry`.

The initial v2 observation bag omitted `/cloud_registered` and therefore has no
ICP output. The recheck below corrected that omission and passed:

```bash
ros2 bag record -o ~/rosbags/fixed_map_static_2026-08-10_v2/recheck \
  /cloud_registered \
  /fast_lio/odometry \
  /fixed_map_localization/odometry \
  /rtk/fix /rtk/heading /rtk/diagnostics \
  /tf /tf_static
```

`~/rosbags/fixed_map_static_2026-08-10_v2/recheck` results:

| Metric | Result | Status |
| --- | ---: | --- |
| Duration | `252.035 s` | Pass |
| ICP outputs | `239` at `~0.95 Hz` | Pass |
| ICP fitness | `0.014688-0.019772` | Pass |
| Candidate x span | `0.014216 m` | Pass |
| Candidate y span | `0.011843 m` | Pass |
| Candidate yaw span | `0.048 deg` | Pass |
| FAST-LIO x/y span | `0.008273/0.009751 m` | Pass |
| FAST-LIO yaw span | `0.049 deg` | Pass |
| RTK fix messages | `117`, all quality `4` | Pass |
| RTK latitude span | `2.755e-7 deg` (`~3.1 cm`) | Pass |
| RTK longitude span | `1.710e-7 deg` (`~1.7 cm`) | Pass |
| RTK altitude span | `0.0464 m` | Pass |
| NTRIP diagnostics | connected, reconnects `1`, last error empty | Pass |
| Wheel speed | `0.0 m/s` for all `4093` messages | Pass |
| `/tf` map owner | no `map -> odom` transforms | Pass |

The localizer received `/cloud_registered` and published continuous candidate
output with no inferred rejection: all 239 finite fitness values were below
the then-configured `0.30` threshold and also below the current `0.10` threshold.
The only dynamic input that was recorded, wheel odometry, remained at
zero, so this is a static-only pass and not evidence of motion tracking.

The RTK heading topic contained only 18 messages over the bag. Its inter-message
interval reached `40.87 s` and its reported yaw span was `3.25 deg`; heading is
therefore not accepted as an independent high-rate TF constraint. The RTK fix
and NTRIP diagnostics are nevertheless stable for this static sample.

The bag also confirms that `/tf` contained only `odom -> base_link`; no
`map -> odom` TF was accidentally published. It may proceed to a manual
low-speed dynamic candidate test, still with `publish_tf:=false`.

## Manual Low-Speed Dynamic Candidate Test

Keep `publish_tf:=false`, all Nav2 nodes stopped, and all vehicle motion under
manual control. Start from the v2 map origin and use the same launch arguments
as the static check. Drive at low speed through mapped areas in this order:

1. Straight ahead `5-10 m`, then stop for `10 s`.
2. Reverse toward the start area, then stop for `10 s`.
3. Left and right turns, returning to the starting heading.
4. A low-speed figure-eight through feature-rich mapped space.
5. An independent route not identical to the capture path.

Record every input and candidate output:

```bash
ros2 bag record -o ~/rosbags/fixed_map_dynamic_2026-08-10_v2/candidate \
  /livox/lidar /livox/imu \
  /fast_lio/odometry /odometry/filtered \
  /cloud_registered /fixed_map_localization/odometry \
  /wheel/odometry/twist \
  /rtk/fix /rtk/heading /rtk/diagnostics \
  /tf /tf_static
```

The candidate passes only if ICP output remains continuous, fitness stays at or
below `0.10`, no repeated rejection occurs, and its `map -> odom` candidate has
no meter-scale or large-angle jumps. Compare the candidate alignment against
the map in RViz before considering TF publication.

### First Dynamic Candidate Result

`~/rosbags/fixed_map_dynamic_2026-08-10_v2/candidate` is useful diagnostic
evidence but does not pass the TF enable gate:

| Metric | Result | Status |
| --- | ---: | --- |
| Duration | `272.160 s` | Recorded |
| ICP outputs | `260`, no gap above `1.164 s` | Pass |
| ICP fitness | median `0.02058`, maximum `0.29972` | Marginal |
| Fitness above `0.10` | `4` outputs | Fail |
| Candidate planar step | maximum `0.1289 m` | Fail |
| Candidate vertical step | maximum `0.2886 m` | Fail |
| Candidate vertical steps above `0.10 m` | `13` | Fail |
| Start/end candidate delta | `0.0039/0.0026/-0.0054 m`, `0.0076 deg` | Pass |
| RTK/NTRIP | quality `4`, connected, no recorded error | Pass |
| `/tf` map owner | no `map -> odom` transforms | Pass |

Two strong point-turn intervals occurred at approximately `64.3-68.9 s` and
`69.2-73.8 s` after bag start. During these intervals, the one-second IMU
acceleration-magnitude standard deviation was approximately `1.05 m/s^2`,
compared with approximately `0.39 m/s^2` for other moving samples. One point
turn produced fitness `0.1504` and a `0.1809 m` candidate translation step,
including a `0.176 m` vertical correction. The visible chassis vibration did
therefore affect fixed-map matching.

The point turns are not the only issue. Other route locations produced fitness
spikes of `0.2997`, `0.1092`, and `0.1267`, and the candidate generated repeated
vertical corrections while driving. A smoother recording is required, but a
smoother point turn alone is not sufficient evidence that these route-dependent
or six-degree-of-freedom ICP effects are resolved.

Repeat the test as a new bag rather than replacing this evidence. Use smooth
rolling turns instead of point turns where the site permits, keep speed below
the first run's approximately `1.19 m/s` peak, avoid abrupt forward/reverse
transitions, and stop for at least `10 s` before and after each route section.
Record `/odometry/filtered` successfully in the repeat; it was requested but is
absent from the first candidate bag. The repeat must still cover the independent
route and the map areas where the first run produced fitness spikes.

### Smooth Dynamic Recording Result

`~/rosbags/fixed_map_dynamic_2026-08-10_v2/candidate_smooth` demonstrates smoother
manual driving, but it is not an ICP validation bag:

| Metric | Result | Status |
| --- | ---: | --- |
| Duration | `687.096 s` | Recorded |
| Peak wheel linear speed | `0.908 m/s` | Improved |
| Strong point-turn duration | `1.83 s` | Improved |
| Moving vibration metric | median `0.248`, mean `0.273 m/s^2` | Improved |
| `/odometry/filtered` | `20613` at `30 Hz` | Recorded |
| `/fast_lio/odometry` | topic absent | Invalid |
| `/cloud_registered` | topic absent | Invalid |
| Fixed-map ICP output | topic registered, `0` messages | Invalid |

The first dynamic bag had approximately `9.27 s` of strong point turns, a peak
speed of `1.187 m/s`, and a moving one-second vibration mean of approximately
`0.388 m/s^2`. The smooth recording reduced all three measures, so its driving
profile is preferable for the next attempt.

However, the local EKF had no recorded FAST-LIO pose correction. While the
vehicle was stationary for the first `150 s`, `odom -> base_link` yaw drifted
`54.6 deg`; during the final stationary period it drifted another `65.2 deg`.
The `/tf` output exactly matched `/odometry/filtered`, confirming that this was
the live local estimate rather than a bag-analysis artifact. This recording must
not be used to approve local localization or fixed-map TF publication.

RTK fixes remained quality `4`, but NTRIP disconnected and recovered twice near
the end of the bag. Recovery completed and fixes continued, although the
diagnostic `last_error` retained the most recent connection error. Dual-antenna
heading remained too sparse for a high-rate constraint.

Before driving another route, verify all required publishers while stationary:

```bash
ros2 topic hz /fast_lio/odometry
ros2 topic hz /cloud_registered
ros2 topic hz /fixed_map_localization/odometry
ros2 topic hz /odometry/filtered
ros2 topic echo /fixed_map_localization/odometry --once
```

Expected rates are approximately `10 Hz`, `10 Hz`, `1 Hz`, and `30 Hz`. Confirm
that the ICP message frames are `map` and `odom`, its fitness is finite and below
`0.10`, and the stationary EKF yaw is not continuously drifting before starting
the next bag. Record to a new directory and stop immediately if any required
topic has zero messages.

### Complete Smooth Dynamic Candidate Result

`~/rosbags/fixed_map_dynamic_2026-08-10_v2/candidate_smooth_complete` contains the
complete localization chain and confirms that the previous missing topics were
an operational startup omission. It still does not pass the TF enable gate:

| Metric | Result | Status |
| --- | ---: | --- |
| Duration | `442.860 s` | Recorded |
| FAST-LIO/cloud | `4429/4429`, approximately `10 Hz` | Pass |
| Local EKF/TF | `13287/13287`, approximately `30 Hz` | Pass |
| ICP outputs | `419`, one `2.089 s` gap | Marginal |
| ICP fitness | median `0.01980`, maximum `0.24430` | Marginal |
| Fitness above `0.10` | `6` outputs | Fail |
| Candidate planar step | maximum `0.1312 m` | Fail |
| Candidate vertical step | maximum `0.2632 m` | Fail |
| Candidate vertical steps above `0.10 m` | `17` | Fail |
| Start/end candidate delta | `0.0012/0.0041/-0.0129 m`, `-0.0241 deg` | Pass |
| FAST-LIO start/end planar delta | `0.0812 m` | Pass |
| Local EKF start/end planar delta | `0.0804 m` | Pass |
| RTK start/end planar delta | `0.0900 m` | Pass |
| RTK/NTRIP | quality `4`, continuously connected | Pass |
| `/tf` map owner | no `map -> odom` transforms | Pass |

The driving profile was smoother than both earlier recordings. Peak wheel speed
was `0.803 m/s`, no point-turn segment was detected, and the moving one-second
acceleration-magnitude standard deviation had median `0.256` and mean
`0.270 m/s^2`. FAST-LIO and the local EKF remained close: sampled planar
difference had median `0.0198 m` and maximum `0.0618 m`; sampled yaw difference
had median `0.025 deg` and maximum `1.35 deg`.

The dynamic ICP problem nevertheless persisted. It generated repeated vertical
corrections, including `17` steps above `0.10 m` and a maximum of `0.263 m`, plus
one `0.131 m` planar step. Fitness did not identify these corrections: the
correlation between fitness and vertical step magnitude was approximately
`0.016`. Vibration had only moderate correlation with vertical step magnitude
(`~0.36`), and large corrections occurred during ordinary rolling motion and
straight driving. Repeating the same configuration with another smooth route is
therefore not the next useful experiment.

The candidate returned close to its starting alignment and was stable during the
final `10 s` stop, so ICP did not permanently diverge. However, publishing its
accepted six-degree-of-freedom estimates would inject transient vertical and
planar changes into `map -> odom`. Keep `publish_tf:=false` until the localizer is
adapted and replay-tested for the ground-vehicle motion model. Candidate fixes
to evaluate include constraining or rejecting implausible z/roll/pitch updates
and adding per-update translation and rotation gates in addition to fitness.

### Ground-Vehicle Constraint Replay

The localizer was changed to preserve the first accepted map-to-odom z, roll, and
pitch, while allowing only x, y, and yaw to vary afterward. It also now rejects
post-alignment updates whose planar translation exceeds `0.10 m` or whose yaw
step exceeds `0.0872665 rad` (`5 deg`). The default fitness threshold is now
`0.10`, matching the dynamic acceptance criterion. The first accepted result is
also bounded to within `1.0 m` and `30 deg` of the configured startup estimate.
After applying the planar constraint, the node recomputes nearest-neighbor
fitness for the actual candidate it will publish. Rejected updates leave the
previous estimate unchanged.

An isolated replay of the complete smooth candidate bag at equivalent update
timing produced:

| Metric | Constrained replay result | Status |
| --- | ---: | --- |
| ICP output messages | `395` | Pass |
| Constrained fitness | `0.01876-0.09917` | Pass |
| Fitness above `0.10` | `0` | Pass |
| z span | `0.000 m` | Pass |
| Planar step maximum | `0.0792 m` | Pass |
| Planar steps above `0.10 m` | `0` | Pass |
| Yaw step maximum | `0.0939 deg` | Pass |
| Start/end yaw delta | `0.0671 deg` | Pass |

This is an offline algorithm result, not live TF approval. The replay was run in
an isolated ROS domain with `publish_tf:=false`; the existing vehicle processes
were not stopped. The fixed-map localizer must still be observed on the vehicle
with the corrected Mid-360 static transform before `publish_tf:=true` is
considered.

### Extrinsic Measurement Impact

The operator measurement changes `base_link -> livox_frame` from the old
`[0.35, 0.00, 0.45] m` to `[0.17, 0.00, 0.48] m`. Because the same vehicle and
Mid-360 generated both the PCD and the registered cloud, this mismatch does not
by itself invalidate the direct cloud-to-PCD ICP coordinate alignment. It does
invalidate the vehicle-center interpretation of the local TF during rotations:
the old x lever arm was wrong by `0.18 m`, which can create centimetre-scale to
approximately `0.18 m` position errors depending on yaw motion. The static TF
launch has therefore been corrected, but the map must be rechecked from the
same start pose after deployment.

The follow-up measurement confirmed the right-side Ant1 at
`base_link -> rtk_link = [-0.17, -0.325, 0.70] m`. The explicitly enabled
global-localization launch defaults now use this Ant1 transform. This RTK
extrinsic does not affect the current ICP result because RTK is not an ICP input
or a `map -> odom` TF owner, but it is required for later earth-to-map datum
calibration.

### Live Constrained Recheck

The requested path was
`~/rosbags/fixed_map_dynamic_2026-08-10_v2/candidate_constrained_recheck1`.
The created bag is at
`~/rosbags/fixed_map_dynamic_2026-08-10_v2/candidate_constrained_recheck`.
It was created at the reported test time and contains the complete runtime
chain:

| Metric | Result | Status |
| --- | ---: | --- |
| Duration | `92.895 s` | Recorded |
| FAST-LIO/cloud | `902/902`, approximately `10 Hz` | Pass |
| Local EKF/TF | `2787/2787`, approximately `30 Hz` | Pass |
| ICP outputs | `87` | Pass |
| ICP fitness | `0.01803-0.03053` | Pass |
| Fitness above `0.10` | `0` | Pass |
| Planar step maximum | `0.00834 m` | Pass |
| Planar steps above `0.10 m` | `0` | Pass |
| z span | `0.000 m` | Pass |
| roll span | `0.00001 deg` | Pass |
| pitch span | `0.00001 deg` | Pass |
| yaw step maximum | `0.0191 deg` | Pass |
| RTK/NTRIP | 50 quality-4 fixes, connected, no error | Pass |
| `/tf` map owner | no `map -> odom` transform | Pass |

The only continuity exception was one ICP interval of `2.086 s`; the other
intervals were approximately `0.97-1.14 s`. This is a recheck pass for the
accepted candidate quality and ground-vehicle constraint, but not yet a TF
enable pass because the candidate output was not continuous at the configured
approximately `1 Hz` rate.

The route included the parking safety point near a roughly `3 deg` slope. In
the slope-classified ICP samples, FAST-LIO pitch changed by approximately
`5-6 deg` relative to the initial flat segment, and moving vibration increased
from a flat-segment mean of approximately `0.059` to a slope-segment mean of
approximately `0.265 m/s^2`. The fixed-map candidate remained stable: slope
fitness median was `0.02362` versus `0.01907` on flat samples, with maximum
planar step `0.00834 m`. The observed FAST-LIO pitch change is a sensor/vehicle
motion observation and is not a direct survey of the exact road grade, but it
confirms that local motion can carry the slope while map-to-odom remains planar.

Therefore the approximately `3 deg` parking slope does not invalidate this
test. It would matter only if the map-to-odom owner were allowed to absorb the
vehicle's pitch, roll, or height changes. The current ground-vehicle constraint
correctly keeps those changes in `odom -> base_link -> livox_frame` and does not
publish them as global map corrections.

The final candidate delta was `-0.0077/-0.0030 m` in x/y and `-0.0075 deg` yaw.
The FAST-LIO and local EKF start/end planar changes were approximately `0.172 m`
over this short route, consistent with the vehicle moving and returning near,
but not exactly to, its start pose. RTK/NTRIP remained stable throughout.

### TF Continuity Recheck

`~/rosbags/fixed_map_dynamic_2026-08-10_v2/candidate_tf_continuity_recheck`
provides the final candidate continuity evidence. It contains a complete 85.128
second route with stationary, moving, and final stationary samples:

| Metric | Result | Status |
| --- | ---: | --- |
| ICP outputs | `80` | Pass |
| ICP interval | `0.966-1.136 s` | Pass |
| ICP intervals above `1.2 s` | `0` | Pass |
| Fitness | `0.01847-0.03300` | Pass |
| Fitness above `0.10` | `0` | Pass |
| Planar step maximum | `0.01367 m` | Pass |
| Planar steps above `0.10 m` | `0` | Pass |
| z, roll, pitch span | `0.000 m`, `0.000003 deg`, `0.000002 deg` | Pass |
| Yaw step maximum | `0.0219 deg` | Pass |
| Candidate start/end delta | `+0.0113/-0.0026 m`, `-0.0222 deg` | Pass |
| Local EKF TF gap maximum | `0.0447 s` | Pass |
| FAST-LIO gap maximum | `0.1474 s` | Pass |
| RTK/NTRIP | 43 quality-4 fixes, connected, no error | Pass |
| Candidate `/tf` owner | no `map -> odom` transform while observing | Pass |

This satisfies the candidate continuity requirement and authorizes a constrained
TF-publish observation. It does not authorize Nav2 or autonomous motion. Start
the fixed-map localizer as the only `map -> odom` owner, with all global RTK TF
owners, Nav2 nodes, and vehicle command publishers stopped:

```bash
ros2 launch fast_lio fixed_map_localization.launch.py \
  publish_tf:=true \
  initial_x_m:=0.0 initial_y_m:=0.0 initial_yaw_rad:=0.0
```

Record `/tf`, `/tf_static`, `/fixed_map_localization/odometry`, registered
cloud, local odometry, wheel twist, and RTK diagnostics during a stationary,
low-speed, and independent-route observation. Verify there is exactly one
`map -> odom` publisher and that the transform age remains bounded. RTK/NTRIP
remained connected in this bag, so an NTRIP loss/recovery observation remains
required before autonomous navigation approval.

## TF Enable Gate

The candidate bag has now demonstrated continuous bounded alignment. The first
controlled `publish_tf:=true` observation is permitted under the constraints
above. Nav2 and autonomous motion remain prohibited until the following runtime
and environmental checks are completed:

- no map-frame jumps on RTK loss or recovery;
- independent route alignment in RViz;
- exactly one active `map -> odom` publisher with bounded transform age;
- a reviewed map version, initial alignment, and capture metadata.

Once accepted, stop every other `map -> odom` publisher and start the same
localizer with `publish_tf:=true`. It then becomes the sole owner.

## First TF-Publish Observation

`~/rosbags/fixed_map_tf_publish_2026-08-10_v2/observation` is the first live
controlled observation with `publish_tf:=true`. It contains the complete
localization chain for `172.788 s`, including 163 fixed-map odometry candidates,
164 `map -> odom` transforms, 5183 local-EKF outputs, 1710 FAST-LIO outputs, and
RTK/NTRIP diagnostics.

| Metric | Result | Status |
| --- | ---: | --- |
| Candidate and `map -> odom` frame pair | `map -> odom` | Pass |
| Candidate samples represented by identical `map -> odom` TF | `163/163` | Pass |
| Additional `map -> odom` TF samples | `1` initial transform only | Pass |
| Conflicting duplicate `map -> odom` stamps | `0` | Pass |
| ICP fitness | `0.02002-0.06068` | Pass |
| Fitness above `0.10` | `0` | Pass |
| Planar update maximum | `0.07478 m` | Pass |
| Planar updates above `0.10 m` | `0` | Pass |
| z span | `0.000 m` | Pass |
| Yaw update maximum | `0.10525 deg` | Pass |
| Local EKF TF maximum interval | `0.04576 s` | Pass |
| FAST-LIO maximum interval | `0.10567 s` | Pass |
| RTK | `53` fix messages, all quality `4` | Pass |
| NTRIP diagnostics | connected, empty last error | Pass |
| Candidate and `map -> odom` maximum interval | `2.20003 s` | Fail |

The sole continuity failure occurred about `18.66-20.86 s` after bag start. The
candidate immediately before and after the gap had fitness `0.03288` and
`0.03524`, respectively, and differed by only `0.0101 m` in the plane and
`0.0328 deg` yaw. This is not a pose jump, but it exceeds the required `1.2 s`
publication interval. The recorded TF path confirms that the fixed-map output
was published as `map -> odom`, and no numerical evidence of a competing owner
is present in the bag; rosbag data cannot identify the originating ROS node.

Do not approve Nav2, autonomous motion, or sustained production use of
`publish_tf:=true` from this observation alone. Repeat a controlled TF-publish
observation after diagnosing the single missed ICP/localizer cycle, and require
no `map -> odom` interval above `1.2 s`. An NTRIP loss/recovery observation also
remains required because this bag stayed connected throughout.

## Performance-Governor TF-Publish Repeat

The RK3588 CPU cores were locked to the `performance` governor before recording
`~/rosbags/fixed_map_tf_publish_2026-08-10_v2/observation_repeat`. The complete
`179.732 s` bag contains 168 fixed-map candidates and 168 `map -> odom` TF
transforms, plus 1797 FAST-LIO and 5388 local-EKF outputs.

| Metric | Result | Status |
| --- | ---: | --- |
| Candidate samples represented by identical `map -> odom` TF | `168/168` | Pass |
| Extra or conflicting `map -> odom` samples | `0` | Pass |
| ICP fitness | `0.01932-0.06120` | Pass |
| Fitness above `0.10` | `0` | Pass |
| Planar update maximum | `0.07010 m` | Pass |
| Planar updates above `0.10 m` | `0` | Pass |
| z span | `0.000 m` | Pass |
| Yaw update maximum | `0.08979 deg` | Pass |
| FAST-LIO maximum interval | `0.11099 s` | Pass |
| Local EKF TF maximum interval | `0.04981 s` | Pass |
| RTK | `56` fix messages, all quality `4` | Pass |
| NTRIP diagnostics | connected, empty last error | Pass |
| Candidate and `map -> odom` maximum interval | `2.09994 s` | Fail |

The missed publication was about `69.12-71.22 s` after bag start. Its bounding
accepted candidates had fitness `0.03656` and `0.03622`, differed by `0.0127 m`
the bounded-age requirement still fails.

Locking CPU frequency was therefore not sufficient to remove the failure. The
localizer advances `last_update_` before running ICP and publishes only after an
accepted ICP result. A raw fitness, constrained-fitness, or motion-gate rejection
therefore consumes an update slot and produces approximately a two-second TF
gap at the configured `1 Hz` update rate, even while FAST-LIO and the local EKF
remain continuous. This bag alone cannot distinguish the rejection reason
because the node does not publish rejection diagnostics.

Before another acceptance observation, make rejected-update reason/count
observable and decouple publication of the last accepted `map -> odom` transform
from acceptance of a new ICP estimate. The heartbeat must start only after the
first accepted alignment, retain the last accepted transform on rejection, and
publish at a bounded rate. Revalidate that behavior in isolated replay before a
third live TF-publish observation. Nav2 and autonomous motion remain blocked;
an RTK/NTRIP loss/recovery observation is still required.

### Heartbeat Implementation

`fixed_map_localizer` now has a `tf_heartbeat_rate_hz` parameter, defaulting to
`5.0 Hz`, and runs its ICP subscription and heartbeat in distinct callback
groups on a two-thread executor. It emits no `map -> odom` TF before the first
accepted ICP alignment. After that first acceptance, it republishes the last
accepted transform at the heartbeat rate, including while an ICP result is
rejected. `max_tf_heartbeat_age_s` defaults to `2.5 s`; after that age without
an accepted ICP result, the node stops refreshing the transform and continues
to emit an error-level `heartbeat_stale` diagnostic. Before first alignment it
continuously reports `awaiting_initial_alignment`. It does not publish heartbeat messages on
`/fixed_map_localization/odometry`; that topic remains an accepted-ICP evidence
stream at approximately `update_rate_hz`.

Accepted ICP odometry retains the source cloud acquisition timestamp. Its
corresponding `map -> odom` TF uses the publication timestamp, like the
heartbeat, so a completed ICP callback cannot insert an older transform after a
newer heartbeat sample.

## Heartbeat TF-Publish Observation

`~/rosbags/fixed_map_tf_publish_2026-08-10_v2/observation_heartbeat` is the
first controlled live observation with the 5 Hz heartbeat and localizer
diagnostics enabled. It is `108.423 s` long and records the complete TF,
candidate, local-EKF, FAST-LIO, RTK/NTRIP, and diagnostic chain.

| Metric | Result | Status |
| --- | ---: | --- |
| Candidate outputs | `102` | Pass |
| `map -> odom` TF transforms | `644` | Pass |
| Candidate samples represented by identical TF | `102/102` | Pass |
| Additional TF samples | `542` heartbeat transforms | Pass |
| TF arrival interval | median `0.19992 s`, maximum `0.20242 s` | Pass |
| TF positive header-stamp interval maximum | `0.44610 s` | Pass |
| TF gaps above `1.2 s` | `0` | Pass |
| ICP candidate interval maximum | `1.99914 s` | Expected rejection slot |
| ICP fitness | `0.02095-0.04097` | Pass |
| Fitness above `0.10` | `0` | Pass |
| Planar update maximum | `0.02383 m` | Pass |
| Yaw update maximum | `0.06095 deg` | Pass |
| z span | `0.000 m` | Pass |
| Localizer diagnostics | `102` accepted, `1` raw ICP rejection | Observed |
| Constrained fitness, motion, initial gate rejections | `0/0/0` | Pass |
| `heartbeat_stale` diagnostics | `0` | Pass |
| FAST-LIO maximum interval | `0.10504 s` | Pass |
| Local EKF maximum interval | `0.04010 s` | Pass |
| RTK/NTRIP | 46 quality-4 fixes, connected, no error | Pass |

The raw ICP rejection explains the otherwise two-second candidate interval. The
heartbeat retained and republished the last accepted transform, so TF consumers
did not see a stale `map -> odom`; this validates the primary heartbeat behavior.

The recorded TF header timestamps nevertheless regress once by `0.24609 s`.
The initial heartbeat implementation published current-time heartbeats in
parallel with accepted ICP TF stamped at the older source-cloud time. The source
has since been corrected: ICP odometry continues to retain source-cloud time for
evidence, while its corresponding TF uses publication time. Rebuild and repeat
an isolated replay plus live TF observation before claiming monotonic TF header
timestamps.

The vehicle did not return to its original map pose in this bag. Composing the
candidate `map -> odom` with `odom -> base_link` at the first and final accepted
samples gives `+0.1855 m` x, `+0.1912 m` y, `0.2664 m` planar, and `-2.481 deg`
map-to-odom drift, but it is too large to accept as a return-to-parking closure
without an independent surveyed/RViz comparison. Do not classify it as a minor
left offset.

This observation validates heartbeat arrival continuity and rejection
observability, but does not approve Nav2 or autonomous motion. The timestamp
fix, an independent-route return-to-parking closure, and RTK/NTRIP loss-recovery
behavior still require live evidence.

The node now also publishes `/fixed_map_localization/diagnostics`. Each accepted
or rejected ICP attempt reports its event and cumulative `accepted_updates`,
`raw_icp_rejections`, `constrained_fitness_rejections`, `motion_gate_rejections`,
and `initial_gate_rejections` counters. Record this topic in the next isolated
replay and live observation to identify the reason for any retained-transform
heartbeat period.

## Timestamp-Fix Chalk Closure Observation

`~/rosbags/fixed_map_tf_publish_2026-08-10_v2/observation_closure` validates the
publication-timestamp correction and uses physical chalk marks for the start and
finish position comparison. The complete bag is `117.698 s` long.

| Metric | Result | Status |
| --- | ---: | --- |
| Candidate outputs | `110` | Pass |
| `map -> odom` TF transforms | `701` | Pass |
| Candidate transforms represented in TF | `110/110` | Pass |
| TF arrival interval | median `0.19990 s`, maximum `0.20092 s` | Pass |
| TF header timestamp interval | `0.00053-0.20085 s` | Pass |
| TF timestamp regressions | `0` | Pass |
| TF gaps above `1.2 s` | `0` | Pass |
| ICP candidate interval maximum | `2.10357 s` | Expected rejection slot |
| ICP fitness | `0.01799-0.03326` | Pass |
| Fitness above `0.10` | `0` | Pass |
| Planar update maximum | `0.04374 m` | Pass |
| Yaw update maximum | `0.06220 deg` | Pass |
| z span | `0.000 m` | Pass |
| Localizer diagnostics | `110` accepted, `1` raw ICP rejection | Observed |
| Constrained fitness, motion, initial gate, stale rejections | `0/0/0/0` | Pass |
| FAST-LIO/local EKF maximum interval | `0.10573/0.05997 s` | Pass |
| RTK/NTRIP | 46 quality-4 fixes, connected, no error | Pass |
| Chalk closure `map -> base_link` planar delta | `0.05366 m` | Pass |
| Chalk closure x/y delta | `-0.02070/+0.04950 m` | Pass |
| Chalk closure yaw delta | `+4.414 deg` | Review required |

The timestamp fix is verified: candidate odometry keeps source-cloud timestamps,
while its matching TF appears within approximately `55 ms` of that time and the
TF sequence remains monotonic. The single raw ICP rejection caused no TF gap;
the heartbeat maintained the last accepted transform.

The physical chalk-mark position closure is approximately `5.4 cm`, including a
leftward component of approximately `5.0 cm`. This is a credible small
return-to-parking positional error and is substantially better than the prior
unsurveyed `26.6 cm` result. The final yaw remains `4.414 deg` from the starting
mark orientation. Position closure passes this observation, but heading closure
requires an explicit tolerance and a repeat if the parking task requires a
repeatable final orientation.

This establishes controlled `publish_tf:=true` TF continuity, monotonic
publication timestamps, and positional return closure. Nav2 and autonomous
motion remain blocked pending the RTK/NTRIP loss-recovery observation and an
explicit heading-closure requirement for the intended parking task.

## Claimed RTK-Recovery Observation

`~/rosbags/fixed_map_tf_publish_2026-08-10_v2/observation_rtk_recovery` is a
complete `171.467 s` controlled TF observation, but it does **not** contain an
NTRIP loss-and-recovery event. It verifies steady-state behavior only.

| Metric | Result | Status |
| --- | ---: | --- |
| Candidate outputs | `163` | Pass |
| Candidate interval | `0.99799-1.10296 s` | Pass |
| `map -> odom` TF transforms | `1021` | Pass |
| TF arrival interval | median `0.19990 s`, maximum `0.20585 s` | Pass |
| TF timestamp interval | `0.00070-0.20108 s`, no regression | Pass |
| TF gaps above `1.2 s` | `0` | Pass |
| ICP fitness | `0.01778-0.02747` | Pass |
| Planar update maximum | `0.00919 m` | Pass |
| z span | `0.000 m` | Pass |
| Localizer diagnostics | `163` accepted, no events in bag | Pass |
| FAST-LIO/local EKF maximum interval | `0.12119/0.08461 s` | Pass |
| RTK fixes | `75`, all quality `4` | Pass |
| NTRIP state | connected throughout; no session reset | Not a recovery test |
| Candidate `map -> base_link` start/end planar delta | `0.08262 m` | Recorded |
| Candidate `map -> base_link` start/end yaw delta | `-1.488 deg` | Recorded |

`ntrip_connected` is `true` in every recorded diagnostic, `last_error` remains
empty, and `ntrip_session_sec` increases monotonically from approximately `734`

To satisfy that gate, record a fresh bag that contains, in order, an observed
`ntrip_connected=false` diagnostic or NTRIP session reset, the corresponding
diagnostic error/state, recovery to `ntrip_connected=true` with a new session or
increased reconnect counter, and restored quality-4 fixes. Keep the fixed-map
localizer as the sole `map -> odom` owner and verify no TF timestamp regression,
gap above `1.2 s`, jump, or `heartbeat_stale` event across that interval.

## RTCM-Stream Disconnect Recheck

`~/rosbags/fixed_map_tf_publish_2026-08-10_v2/observation_rtk_disconnect_recheck`
is a `267.482 s` true correction-stream interruption observation. The recorded
NTRIP diagnostics do not show a transport state transition, but
`ntrip_session_rtcm_bytes` froze for approximately `70.0 s` and `42.0 s` in two
separate intervals. The network injection therefore stopped RTCM delivery while
leaving the TCP socket open.

| Metric | Result | Status |
| --- | ---: | --- |
| Candidate outputs | `252` | Pass |
| `map -> odom` TF transforms | `1589` | Pass |
| TF arrival interval maximum | `0.22467 s` | Pass |
| TF header interval maximum / regressions | `0.22417 s` / `0` | Pass |
| TF gaps above `1.2 s` | `0` | Pass |
| ICP fitness | `0.01752-0.03426` | Pass |
| Planar update maximum | `0.02632 m` | Pass |
| z span | `0.000 m` | Pass |
| Localizer diagnostics | `252` accepted, `2` raw ICP rejections | Pass |
| `heartbeat_stale` diagnostics | `0` | Pass |
| FAST-LIO/local EKF maximum interval | `0.20636/0.05903 s` | Pass |
| RTK fixes | `106`, all quality `4` | Pass |
| RTCM stalls | `70.0 s`, `42.0 s` | Observed |
| NTRIP state transition/session reset/reconnect increment | none | Fail recovery observability |

The fixed-map localizer is unaffected by the RTCM interruption: no map-frame
jump, TF age failure, timestamp regression, or localizer stale event occurred.
However, the existing NTRIP client treats each socket read timeout as normal
forever. It consequently kept `ntrip_connected=true`, retained an empty error,
localization behavior during correction loss, but not NTRIP recovery behavior.

`mobileess_rtk` now applies `ntrip_rtcm_timeout_sec` (default `3.0 s`): absence
of incoming RTCM beyond that interval raises a correction-stream timeout, marks
NTRIP disconnected, records the error, closes the client, and follows the
existing reconnect path. Diagnostics now also expose `ntrip_rtcm_age_sec`; a
successful reconnection clears the prior error. Rebuild and repeat the injected
test using this version before clearing the recovery gate.

## RTCM Timeout Disconnect-Recovery Observation

`~/rosbags/fixed_map_tf_publish_2026-08-10_v2/observation_rtk_disconnect_recovery`
is the post-timeout implementation recheck. It is a complete `216.613 s` live
injection observation and satisfies the RTK/NTRIP recovery gate.

| Metric | Result | Status |
| --- | ---: | --- |
| Candidate outputs | `205` | Pass |
| `map -> odom` TF transforms | `1285` | Pass |
| TF arrival/header interval maximum | `0.20211/0.20111 s` | Pass |
| TF timestamp regressions or gaps above `1.2 s` | `0/0` | Pass |
| ICP fitness | `0.01591-0.03564` | Pass |
| Planar update maximum | `0.01687 m` | Pass |
| z span | `0.000 m` | Pass |
| Localizer diagnostics | `205` accepted, no stale event | Pass |
| FAST-LIO/local EKF maximum interval | `0.11883/0.04579 s` | Pass |
| RTK fixes after recovery | `73`, all quality `4` | Pass |
| RTCM timeout diagnostic | `NTRIP correction stream timed out` | Pass |
| NTRIP state transitions | `true -> false -> true -> false -> true` | Pass |
| NTRIP reconnect counter | `1 -> 2 -> 3` | Pass |
| NTRIP session resets | `2` | Pass |
| NTRIP errors observed | timeout, caster close, socket timeout | Pass |
| RTK recovery map-to-base start/end planar delta | `0.08565 m` | Recorded |
| RTK recovery map-to-base start/end yaw delta | `-0.822 deg` | Recorded |

The first injected RTCM interruption was observed at approximately `90 s` after
bag start: the correction stream stopped, `ntrip_rtcm_age_sec` passed the `3 s`
timeout, and diagnostics changed `ntrip_connected` from `true` to `false`. A
new NTRIP session appeared about `35 s` later. That session then closed after
about one second; diagnostics again reported disconnected and a final successful
connection followed about six seconds later. The final session continued while
quality-4 RTK fixes resumed. The diagnostics include the expected timeout error,

Across both loss/recovery windows, fixed-map localization remained independent
of RTK transport health: no `map -> odom` jump, TF timestamp regression, TF gap
above `1.2 s`, or `heartbeat_stale` event occurred. This clears the RTK/NTRIP
loss-recovery gate for fixed-map TF publication.

The recovery route's `0.822 deg` start/end yaw difference is favorable, but this
bag does not state that it used the previous chalk heading marks. It cannot by
itself replace the explicit parking-heading tolerance that remains required for
an autonomous parking task.

## Parking Task Acceptance

The parking task specification defines a general parking use case, not a
precision docking operation. Final heading is a soft preference within the
following acceptance bounds:

| Metric | Acceptance limit | Chalk-closure result | Status |
| --- | ---: | ---: | --- |
| Final `map -> base_link` planar position error | `<= 0.10 m` | `0.05366 m` | Pass |
| Final yaw error | `<= 7.0 deg` | `4.414 deg` | Pass |

The `4.414 deg` chalk-closure yaw difference is attributed to manual driving
and mechanical alignment with the physical marks. It is within the declared
general-parking tolerance and is not a localization rejection. This completes
the fixed-map localization acceptance gates for a controlled Nav2 and
autonomous-parking integration test.

This approval is limited to the fixed parking lot, preconfigured goal or safe
parking areas, low-speed operation, and an operator able to take over or stop
the vehicle immediately. It is not approval for unattended production use or
operation outside the mapped boundary. Before commanding motion, confirm that
`fixed_map_localizer` is the sole `map -> odom` owner, the RTK global EKF still
has `publish_tf: false`, static map boundaries and keep-out zones are loaded,
and Nav2 velocity output reaches the STM32 only through the established RK3588
high-level control interface.

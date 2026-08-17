# RTK Dual-Antenna Validation - 2026-08-10

## Scope

- Receiver: UM982, Ant1 right rear, Ant2 left rear.
- Baseline: measured approximately `0.65 m`; receiver observations reported
  approximately `0.62-0.63 m` with `SOL_COMPUTED` and `NARROW_INT`.
- Historical Ant1 extrinsic under test: `base_link -> rtk_link = [-0.30, -0.345, 0.50] m`.
- Follow-up Ant1 measurement: `base_link -> rtk_link = [-0.17, -0.325, 0.70] m`;
  Ant1 is the right-side primary antenna. This supersedes the historical
  translation for future explicitly enabled global-localization launches.
- Bags:
  - `~/rosbags/rtk_dual_antenna_static/static_test_10min`
  - `~/rosbags/rtk_dual_antenna_dynamic/dynamic_test_01`

## RTK And Local Localization Results

| Check | Static result | Dynamic result | Status |
| --- | --- | --- | --- |
| NTRIP and RTK fix | Connected, `quality=4`, no reconnect in capture | Connected, `quality=4`, no reconnect in capture | Pass |
| RTK horizontal span | `0.024 m` north/south, `0.033 m` east/west | `8.176 m` north/south, `4.839 m` east/west during driving | Pass |
| Local EKF pose span | `0.012 m x`, `0.010 m y`, `0.117 deg yaw` | `6.328 m x`, `7.963 m y` | Pass |
| FAST-LIO pose span | `0.012 m x`, `0.009 m y`, `0.062 deg yaw` | `6.276 m x`, `7.929 m y` | Pass |

The static bag duration is `422.595 s`; the dynamic bag duration is `189.016 s`.

## Global TF Result

The RTK-only global EKF must not publish `map -> odom`.

| Check | Static result | Dynamic result | Status |
| --- | --- | --- | --- |
| `map -> odom` translation span | `0.0179 m x`, `0.0190 m y` | `2,530,017.669 m x`, `1,445,193.765 m y` | Fail |
| `map -> odom` yaw span | `154.812 deg` while stationary | `235.227 deg` | Fail |
| `/global/odometry/filtered` yaw span | `154.784 deg` while stationary | `225.973 deg` | Fail |

`/odometry/gps` was emitted in the `odom` frame while containing UTM-scale ENU
coordinates. Fusing it into a `map`-world EKF that publishes `map -> odom`
requires the transform being estimated, creating a coordinate-frame feedback
path. The dynamic replay demonstrates catastrophic divergence.

## Dual-Antenna Heading Result

The receiver produced valid `UNIHEADINGA` examples with `SOL_COMPUTED`,
`NARROW_INT`, a valid baseline, and approximately `0.53-0.59 deg` heading
standard deviation. Direction checks were performed at four vehicle headings.

The bag capture rate is nevertheless insufficient for TF ownership:

| Bag | Heading samples | Maximum sample gap |
| --- | ---: | ---: |
| Static | 23 | `55.156 s` |
| Dynamic | 8 | `62.167 s` |

The heading output must be made continuous and independently validated before
it is used by any global transform owner.

## Decision

- Keep `odom -> base_link -> livox_frame` as the verified runtime TF chain.
- Keep `base_link -> rtk_link` as the verified Ant1 static transform.
- Keep `/rtk/fix`, `/rtk/heading`, `/odometry/gps`, and
  `/global/odometry/filtered` as observation and recording topics.
- Set the RTK global EKF `publish_tf: false`; no runtime process currently
  publishes `map -> odom`.
- Do not start Nav2 against this RTK-only global estimate.

## Required Follow-up

1. Configure UM982 `UNIHEADINGA` output at a verified continuous rate and
   record a new static bag with no multi-second heading gaps.
2. Implement or integrate fixed-map Mid-360 map matching as the sole validated
   `map -> odom` owner.
3. Load a surveyed fixed datum and verify the map axes against earth before
   enabling navigation.

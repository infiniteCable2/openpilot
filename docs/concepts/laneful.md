# Laneful (OP lane lines + E2E)

Laneful is an optional, vehicle-independent lateral mode. It starts disabled and
the `EnableLaneful` IC toggle is persistent on Mici and Tici. It uses only the
two inner lane lines and the E2E path already published in `modelV2`; no car
camera signal or opendbc change is needed.

The controller fits the lateral shift and heading between the E2E path and the
lane midpoint over their common 8–55 m horizon. It preserves the near E2E path
shape and increasingly favors the lane midpoint from 10 to 45 m. A weighted fit
near the speed-dependent preview point converts this virtual path difference
to a **bounded correction of the E2E curvature action**. The model path in
`modelV2.position` is not rewritten. This is a scalar steering approximation
to a virtual target, not a new trajectory optimizer. The spatial fitting idea
was informed by [gm1500's lp-e2e-blend experiment](https://github.com/gm1500/openpilot/tree/79762bcaf8555795e309e630329c7b313d92b48b).

The mode needs both lines with plausible width, sufficient probabilities,
reasonable standard deviations, and a common horizon with the E2E path. A
large lane/E2E disagreement reduces or suppresses its influence. Strong lines
must be continuous for 0.75 s before correction begins. A single bad frame can
hold the previous bounded target for at most 150 ms; further loss makes the
correction decay. Blinker, model lane change, lateral maneuver, disabled
toggle, low speed, or stale model also make it decay. A valid lateral maneuver
plan takes immediate priority over the residual correction while its state
decays. Steering-wheel input does not gate or reset the desired path. This
keeps the path policy independent of the driver's short steering corrections.

The contribution is limited to a nominal 50 cm preview displacement, 0.00045 1/m
of curvature and 0.35 m/s² of added lateral acceleration. The curvature and
acceleration limits can reduce the effective preview displacement below 50 cm,
especially at highway speeds. Entry and release rates are limited, with faster
release. `controlsd` applies it before its existing curvature clip and lateral
controller; `curvatured` still handles its own
actuator correction. `controlsStateIC` logs the Laneful correction, quality
and active state for route review.

Ten focused tests cover the real `modelV2` message layout, fit direction,
confidence and geometry gates, arming, one-frame hold, controlled release,
speed limits, and a delayed straight-lane response. A dry replay of the old
43-segment CUPRA Born route (`d4dd69160a48f11f/00000003--9cfe00cb74`)
found 19,908 active model samples out of 51,485. At those samples, the 90th
percentile magnitude of added lateral acceleration was 0.172 m/s² and the
maximum was 0.35 m/s². This replay used recorded inputs and does not show how
the vehicle would respond. A fresh route and a cautious road comparison are
needed to assess settling, curve tracking, and actual ping-pong behavior.

# Running ORB-SLAM3 on an underwater ROV (RTSP camera streams)

A practical assessment and recommended architecture for feeding this package
from a remotely-operated vehicle's video, with the research literature behind
each recommendation. Short version: **appropriate for near-structure survey/
inspection work with proper in-water calibration and sensor fusion; not as a
stand-alone localizer for open-water or turbid transects.**

---

## 1. Verdict

ORB-SLAM3 is a reasonable choice underwater *as one sensor in a fused estimator*,
not as the primary localizer. It is feature-based (ORB), which tolerates the
lighting artefacts of the marine domain better than direct/photometric methods,
and in the one apples-to-apples underwater benchmark of open-source VIO/SLAM it
was competitive [Joshi 2019]. Follow-up work specifically found ORB-SLAM3
outperforms ORB-SLAM2 and Dual-SLAM in complex/turbid underwater scenes
[UW-Mono-Challenges 2023]. But every serious underwater SLAM system that
achieves robust results does so by **adding non-visual sensors** — SVIn2 augments
visual-inertial with **sonar + pressure depth** precisely because vision-only
degrades [Rahman 2018/2022]; AQUALOC, the reference ROV dataset, ships
**camera + IMU + pressure** for the same reason [Ferrera 2019]. Treat vision as
a source of loop closures and structure-relative pose, and let depth/DVL/AHRS
carry the dead-reckoning.

---

## 2. Why underwater is hard (and what breaks ORB-SLAM3)

| Phenomenon | Effect on ORB-SLAM3 | Mitigation |
| --- | --- | --- |
| **Feature scarcity** (open/mid water) | No texture to track → tracking `LOST` the moment the bottom/structure leaves view | Fly close to structure; fuse with DVL/depth so the estimator survives dropouts |
| **Turbidity / low contrast** | ORB descriptors become ambiguous; localization drift rises with turbidity and, at high turbidity, tracking *fails* [UW-Mono-Challenges 2023] | Image enhancement pre-stage; strong, diffuse lighting; accept a working-visibility envelope |
| **Backscatter from onboard lights** | Illuminated particles ("marine snow") become spurious, view-dependent features → violate the static-world assumption | Diffuse/offset lighting away from the optical axis; RANSAC absorbs a moderate amount, heavy load kills it |
| **Moving light cone** | Shadows/highlights move with the vehicle → non-static appearance, weak frame-to-frame matches | Feature-based helps vs. direct; keep lighting fixed relative to the camera |
| **Refraction through the housing port** | Wrong intrinsics/distortion → systematic reprojection error, scale/shape bias | **Calibrate in water** (Section 3) — the single most common cause of bad underwater runs |
| **Monocular scale ambiguity + drift** | No metric scale; scale drifts over a run | Stereo, or fuse a metric sensor (DVL/depth) to fix scale — see Section 5 |

These are domain properties, not bugs in this port. The diagnostics this package
publishes (`/diagnostics`: tracking state, inlier count, rate) are specifically
useful here — you will watch `RECENTLY_LOST` come and go, and that is expected.

---

## 3. Camera & housing calibration — the #1 gotcha

Refraction at a **flat port** increases the effective focal length by ~25 %
(≈ the 1.33 refractive index of water) and adds a non-radial, depth-dependent
distortion that a pinhole + radial model only approximates [She 2019,
Seegräber 2024]. Consequences:

- **Calibrate the camera *in water*, through the actual housing**, not in air.
  An air calibration will be ~25–33 % wrong in focal length and will bias scale
  and reprojection throughout the run. This is the most frequent reason
  underwater ORB-SLAM "sort of works but drifts."
- **Dome ports** are far friendlier: with the lens entrance pupil at the dome's
  centre of curvature, rays cross the interface orthogonally and refraction ≈ 0,
  so a normal pinhole model holds [She 2019]. If you have the choice, dome > flat.
- **Wide-FOV flat ports** produce distortion beyond what `PinHole` + `k1..k4`
  captures — use ORB-SLAM3's **`KannalaBrandt8`** fisheye model (already built and
  exported by this package) and expect to iterate the calibration.
- For rigorous refractive modelling, an open-source refractive calibration
  toolbox exists [Seegräber 2024]; for most inspection work an in-water
  pinhole/KB8 fit with low RMS reprojection error is sufficient.

Put the resulting intrinsics/distortion, resolution, fps, and (stereo) baseline
into your ORB-SLAM3 `settings.yaml` — see `orb_slam3_ros/config/README.md`.

---

## 4. The RTSP bridge — timestamping is the whole game

Your plan (a separate node that pulls the RTSP stream, stamps frames, and
republishes as `sensor_msgs/Image`) is the right shape. This package consumes any
`Image`/`Imu` topic, so the bridge is the only new piece. The make-or-break
detail is **how you stamp**:

### Vision-only modes (`mono` / `stereo` / `rgbd`): receipt-time is acceptable
ORB-SLAM3 in vision-only mode uses timestamps only for ordering and for the
constant-velocity motion model's `Δt`. Stamping at packet receipt just makes the
pose come out one decode+network latency late (typically 100–500 ms), stamped
consistently. Mapping, loop closure, and RViz are unaffected. This is the
pragmatic default and it works.

### Inertial modes (`IMU_MONOCULAR` / `IMU_STEREO`): receipt-time is NOT acceptable
Camera↔IMU temporal alignment for visual-inertial fusion needs single-digit-
millisecond accuracy. RTSP arrival jitter is tens of ms, compounded by GOP/
decoder buffering — stamping at `now()` makes the inertial modes diverge or
refuse to initialize. If you want VIO you must recover **capture time**, two ways:

1. **RTCP Sender Reports.** The RTSP server periodically emits an SR carrying the
   NTP↔RTP timestamp mapping — i.e. the camera's absolute capture clock for each
   RTP timestamp [RFC 3550, RFC 7273]. GStreamer reconstructs this and attaches a
   `GstReferenceTimestampMeta` to each buffer once synchronized; stamp your ROS
   `Image` from that, not from arrival [Dröge 2022]. Caveat: the first SR can take
   ~5 s to arrive (default SR interval), so hold or receipt-stamp until sync, then
   switch. `rtpjitterbuffer`/`rtpbin` expose the machinery.
2. **Clock discipline over the tether.** Independently, sync the ROV computer's
   clock to topside: **PTP (IEEE 1588)** if the tether switch supports it, NTP as
   a fallback. This keeps the camera-node stamps and the IMU stamps on the same
   timeline.

Bottom line: **use vision-only + receipt-time to start; only invest in RTCP-SR/
PTP timestamping if you specifically want the inertial modes.** Given an ROV
already carries better inertial/velocity aiding (DVL, AHRS) than a webcam IMU,
vision-only ORB-SLAM3 + external fusion (Section 5) is usually the better bet
than ORB-SLAM3's own VIO on jittery RTSP.

### Encoding & transport settings that matter for features
- **Disable B-frames** — frame reordering adds latency and stamping ambiguity.
- **Push bitrate as high as the tether allows.** H.264 blocking artefacts churn
  ORB descriptors; rate-control quality pulsing makes features flicker between
  I- and P-frames, hurting frame-to-frame matching.
- **Prefer constant quality / high GOP quality** over aggressive compression.
- The bridge is a *local* republisher, so run our nodes against it with
  `qos_reliability:=reliable` (not the `sensor_data` best-effort default used for
  hardware drivers) — there's no lossy RF link between the two ROS nodes.
- `gscam2` is a usable off-the-shelf GStreamer→ROS 2 bridge to start from; a
  purpose-built `appsink` node gives you control over the SR-based stamping.

---

## 5. Recommended architecture: ORB-SLAM3 as a sensor, not the localizer

Don't ask vision to do the whole job. An ROV already carries the sensors that
cover vision's blind spots; fuse them:

```
 RTSP camera ─▶ rtsp_bridge (stamp) ─▶ /camera/image ─▶ orb_slam3 (mono/stereo)
                                                             │  ~/odom (pose + covariance)
 DVL  ──────────────────────────────────────────────▶       ▼
 Pressure/depth ───────────────────────────────▶  robot_localization EKF ─▶ fused odom → TF
 AHRS/compass ─────────────────────────────────▶
```

- **Depth (pressure)** gives absolute, drift-free Z — the cheapest, most reliable
  underwater constraint. AQUALOC and SVIn2 both lean on it [Ferrera 2019,
  Rahman 2018]. Fuse it directly.
- **DVL** gives bottom-lock metric velocity — this both **fixes monocular scale**
  and lets the filter dead-reckon through the visual dropouts that *will* happen.
- **AHRS/compass** constrains heading/attitude drift.
- **ORB-SLAM3** then contributes what it's good at: locally accurate,
  structure-relative pose and **loop closures** when there's texture to see.

This is exactly why this branch added a proper covariance model on `~/odom`
(`nav_msgs/Odometry`): feed it into `robot_localization`'s EKF/UKF [Moore 2014]
alongside DVL/depth/AHRS. Two important knobs for this use case, documented in
`orb_slam3_ros/README.md`:
- Use `covariance_mode:=quality` (default) so the filter automatically de-weights
  vision as the inlier count drops, or `g2o` for the true motion-only-BA marginal.
- The `g2o` marginal is **overconfident** (motion-only BA treats landmarks as
  perfectly known), so set `covariance_g2o_scale` ≈ 5–10 before trusting it in a
  filter — otherwise `robot_localization` will mainline pure VO and follow it off
  a cliff during a tracking glitch.

The fully-coupled alternative (sonar + vision + inertial + depth in one optimizer)
is SVIn2 / AQUA-SLAM territory [Rahman 2022, AQUA-SLAM 2025] — better ultimate
accuracy, much more integration effort, and a different codebase. For most ROV
inspection work, ORB-SLAM3-as-a-sensor into `robot_localization` is the pragmatic
90 % solution.

---

## 6. Pre-dive checklist

- [ ] Camera calibrated **in water, through the housing** (dome ≫ flat; KB8 for wide flat ports).
- [ ] `settings.yaml` intrinsics/distortion/fps/baseline match the in-water calibration.
- [ ] Lighting diffuse and offset from the optical axis to limit backscatter.
- [ ] Start **vision-only** (`mono`/`stereo`), receipt-time stamps, `qos_reliability:=reliable`.
- [ ] Depth + DVL + AHRS fused with `~/odom` in `robot_localization`; scale/Z come from those.
- [ ] Watch `/diagnostics` — plan operations around `RECENTLY_LOST` being routine, not fatal.
- [ ] Only pursue RTCP-SR/PTP timestamping + inertial modes if fusion isn't enough.

---

## References

- **Campos, Elvira, Gómez Rodríguez, Montiel, Tardós.** "ORB-SLAM3: An Accurate
  Open-Source Library for Visual, Visual–Inertial and Multi-Map SLAM." *IEEE T-RO*
  37(6), 2021.
- **[Joshi 2019]** B. Joshi, S. Rahman, M. Kalaitzakis, B. Cain, J. Johnson,
  M. Karapetyan, A. Newby, A. Quattrini Li, I. Rekleitis. "Experimental Comparison
  of Open Source Visual-Inertial-Based State Estimation Algorithms in the
  Underwater Domain." *IROS* 2019. arXiv:1904.02215.
- **[Ferrera 2019]** M. Ferrera, V. Creuze, J. Moras, P. Trouvé-Peloux. "AQUALOC:
  An underwater dataset for visual–inertial–pressure localization." *Int. J. of
  Robotics Research* 38(14), 2019. arXiv:1910.14532.
- **[Rahman 2018/2022]** S. Rahman, A. Quattrini Li, I. Rekleitis. "SVIn2: Sonar
  Visual-Inertial SLAM with Loop Closure for Underwater Navigation." *IROS* 2018
  (arXiv:1810.03200); extended as "SVIn2: A multi-sensor fusion-based underwater
  SLAM system," *IJRR* 41(11-12), 2022.
- **[UW-Mono-Challenges 2023]** "Investigation of the Challenges of
  Underwater-Visual-Monocular-SLAM." 2023. arXiv:2306.08738. (ORB-SLAM3 vs.
  ORB-SLAM2/Dual-SLAM in turbid scenes; turbidity → descriptor ambiguity → drift/
  tracking failure.)
- **[She 2019]** M. She, D. Nakath, Y. Song, K. Köser. "Adjustment and Calibration
  of Dome Port Camera Systems for Underwater Vision." *GCPR* (LNCS), 2019. (Dome
  vs. flat port refraction; entrance-pupil alignment.)
- **[Seegräber 2024]** F. Seegräber et al. "A Calibration Tool for Refractive
  Underwater Vision." 2024. arXiv:2405.18018. (Open-source refractive camera/
  stereo/housing calibration.)
- **[AQUA-SLAM 2025]** "AQUA-SLAM: Tightly-Coupled Underwater Acoustic-Visual-
  Inertial SLAM with Sensor Calibration." 2025. arXiv:2503.11420.
- **[Moore 2014]** T. Moore, D. Stouch. "A Generalized Extended Kalman Filter
  Implementation for the Robot Operating System" (`robot_localization`). *IAS-13*,
  2014.
- **[Kannala 2006]** J. Kannala, S. Brandt. "A Generic Camera Model and Calibration
  Method for Conventional, Wide-Angle, and Fish-Eye Lenses." *IEEE T-PAMI* 28(8),
  2006. (The `KannalaBrandt8` model for wide flat ports.)
- **[RFC 3550]** Schulzrinne et al. "RTP: A Transport Protocol for Real-Time
  Applications." IETF, 2003. (RTCP Sender Reports; NTP↔RTP mapping.)
- **[RFC 7273]** Williams, Jerome, Weber. "RTP Clock Source Signalling." IETF, 2014.
- **[Dröge 2022]** S. Dröge. "Instantaneous RTP synchronization & retrieval of
  absolute sender clock times with GStreamer." coaxion.net, 2022. (Practical
  RTCP-SR / `GstReferenceTimestampMeta` recovery of capture time.)

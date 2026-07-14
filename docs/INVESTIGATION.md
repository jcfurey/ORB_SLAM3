# ORB-SLAM3 Fork — Full-Repository Investigation

> **Scope.** Core SLAM logic, the ROS 2 integration, and the mathematics of
> localization, mapping, loop/merge correction, IMU preintegration, bundle
> adjustment, and the pose-covariance feature.
> **Branch** `lyrical` (base commit `0300e27`) · **Date** 2026-07-14 ·
> **Upstream baseline** UZ-SLAMLab/ORB_SLAM3 `master`, with ~25 fixes already
> applied (see [`docs/ORB_SLAM3_AUDIT.md`](ORB_SLAM3_AUDIT.md),
> [`docs/THREAD_SAFETY.md`](THREAD_SAFETY.md)).

## How this investigation was run

Twelve independent investigators each (a) explained how one subsystem actually
works *from the real code* and (b) audited it for correctness, numerical,
thread-safety, and integration defects. Every finding was then adversarially
cross-examined by two independent skeptics — a **correctness** lens that
re-read the cited code and tried to *refute* the failure scenario, and a
**literature/math** lens that checked the cited paper, equation, or
mathematical claim (with web lookups where needed). Findings refuted by
verification were dropped: **49 of 53 survived** and are merged into the **47
distinct items** below. Coverage: Tracking, LocalMapping (+ VI init),
LoopClosing/Merging (+ GBA), the Optimizer pose/BA/graph maths, the
IMU-preintegration maths (`ImuTypes`/`G2oTypes`), the ORB extract/match
front-end, two-view/PnP/Sim3/camera geometry, the Frame/KeyFrame/MapPoint/
Map/Atlas/System data model, the ROS 2 node, the new `~/odom` covariance
frame-transform maths, and the build/packaging (including an adversarial
re-review of the ~25 already-applied fixes).

This is **static + literature analysis** — the SLAM system was not run against
datasets in this pass. Every finding carries an exact `file:line`, a concrete
trigger, and a suggested fix; references are collected at the end. Where
adversarial verification was split (roughly one confirm / one refute) or
investigator confidence was low, that is called out inline.

---

## Executive summary

The `lyrical` fork is, on the whole, a faithful and now materially hardened ORB-SLAM3: the front-end, tracking state machine, local/global BA, loop-and-merge correction, and Atlas multi-map machinery all follow the published algorithms, and the ~25 applied fixes plus the restored `System::Shutdown()` wait loop close several genuine crash and race classes. However, this full-repository pass surfaced **47 distinct correctness findings that survived adversarial verification**, including eight High-severity defects that are live crashes, hangs, or silent estimator corruption on supported sensor configurations (visual-inertial monocular, stereo-fisheye, and the ROS TF tree). Two themes dominate: (1) the multi-camera (KannalaBrandt fisheye) paths remain the least-tested surface — several out-of-bounds indexes, a wrong-camera projection, and a mis-wired calibration reader all cluster there; and (2) the newly added pose-covariance feature is *mathematically implemented correctly as a conditional covariance* but is over-confident, non-metric in monocular, absent in inertial mode, and discontinuous at its fallback boundary. A new engineer should treat pure-visual stereo/RGBD as the most trustworthy path today, and the VI-monocular and fisheye paths as needing the fixes below before field use.

| Severity | Count |
|----------|-------|
| High     | 8     |
| Medium   | 21    |
| Low      | 18    |
| **Total**| **47**|

*Several findings are marked where the underlying verification was only "plausible" (roughly one confirming and one refuting adversarial pass) or where investigator confidence was low; these are called out inline.*

## Architecture — how the system works

### Front-end (extraction and matching)

An incoming image becomes oriented ORB features in `ORBextractor::operator()` (`ORBextractor.cc:1089`). `ComputePyramid` (`ORBextractor.cc:1173`) builds `nlevels` resized images, each laid into a temp `Mat` padded by `EDGE_THRESHOLD=19` (`BORDER_REFLECT_101`) so BRIEF sampling stays valid at borders. `ComputeKeyPointsOctTree` (`ORBextractor.cc:782`) tiles each level into ~35 px cells, runs FAST at `iniThFAST` and falls back to `minThFAST` on empty cells, with `nCols`/`nRows` now clamped `>=1` (`803-804`) to prevent divide-by-zero on tiny top levels. `DistributeOctTree` (`555`) homogenizes the keypoints through a quadtree (`nIni` clamped `>=1` at `560`, recursive `DivideNode` at `480`), keeping the highest-response keypoint per node. Orientation comes from the intensity centroid `IC_Angle` (`76`, `fastAtan2(m01,m10)`), and `computeOrbDescriptor` (`107`) steers the 256 BRIEF tests by that angle.

Matching lives in `ORBmatcher`. `DescriptorDistance` (`ORBmatcher.cc:2058`) is an 8-word SWAR popcount of the XOR. Search windows scale with octave (`radius = th*mvScaleFactors[level]`, candidates restricted to `[level-1,level]`), the Lowe ratio `mfNNratio` and thresholds `TH_LOW=50`/`TH_HIGH=100` gate acceptance, and every matcher builds a `HISTO_LENGTH=30` rotation-consistency histogram whose three fullest bins are found by `ComputeThreeMaxima` (`2012`); matches outside those bins are dropped. `SearchForTriangulation` (`907`) matches unmatched features per shared BoW node under an epipolar/epipole constraint, and `Fuse` (`1148`) reprojects map points and merges observations under chi-square gates (5.99 mono / 7.8 stereo).

### Localization (tracking)

`Tracking::Track()` (`src/Tracking.cc:1799`) runs on the caller/ROS thread. It guards against a LocalMapping bad-IMU flag (`1810`) and non-monotonic or >1 s timestamp jumps that reset or split inertial maps (`1825-1860`), preintegrates IMU via `PreintegrateIMU` (`1879`/`1624`), takes `mMutexMapUpdate` (`1891`), and computes `mbMapUpdated` from the map-change index (`1897-1901`). If uninitialized it runs Stereo/Monocular initialization; otherwise it seeds a pose. With `mState==OK` it selects `TrackReferenceKeyFrame` (BoW + motion-only `PoseOptimization`) when there is no velocity/IMU, else `TrackWithMotionModel` (`2851`): `UpdateLastFrame`, then either `PredictStateIMU` (`2862`, Forster preintegration [R3]) or the constant-velocity model, `SearchByProjection`, and `PoseOptimization` (`2906`).

On failure the state machine (`1964-1981`, `2147-2169`) demotes `OK→RECENTLY_LOST` (if >10 KFs) or `LOST`; RECENTLY_LOST recovers through `PredictStateIMU` (inertial, `1994`) or `Relocalization` (visual, `2008`), giving up after 5 s (IMU) / 3 s (visual). `Relocalization` (`3606`) runs `MLPnPsolver` RANSAC per candidate, then guided `SearchByProjection` refinement, accepting at `nGood>=50`. Given a seed, `TrackLocalMap` (`2946`) builds the local map and runs `PoseOptimization` or the inertial `PoseInertialOptimizationLast{Frame,KeyFrame}` (`2967-2990`) before `NeedNewKeyFrame`/`CreateNewKeyFrame`. The published pose is `mCurrentFrame.GetPose()` returned by `GrabImage*`; its 6×6 covariance is `H^-1` from the motion-only Fisher information built inside `Optimizer::PoseOptimization` (`Optimizer.cc:1123-1167`), set only for the visual optimizer.

### Mapping (local mapping and VI initialization)

`LocalMapping::Run()` (`src/LocalMapping.cc:64`) is the mapping-thread loop. When `CheckNewKeyFrames()` has work and `!mbBadImu` it runs `ProcessNewKeyFrame()` (`298`) → `MapPointCulling()` (`347`, found-ratio `<0.25` / too-few-observation retirement) → `CreateNewMapPoints()` (`389`, triangulation gated on baseline, parallax, positive depth, chi-square 5.991/7.8, and octave-scale consistency) → `SearchInNeighbors()` (`715`, bidirectional `Fuse`) → local BA → optional `InitializeIMU`/VIBA/`ScaleRefinement` → `KeyFrameCulling()` (`903`), then hands the KF to LoopClosing.

Visual-inertial init (`InitializeIMU`, `1174`) collects the temporal KF chain, estimates gravity direction `Rwg` and velocities (`1245-1252`), runs `Optimizer::InertialOptimization`, `ApplyScaledRotation` under `mMutexMapUpdate`, then `FullInertialBA`, and propagates the correction through the spanning tree. `VIBA1`/`VIBA2` re-call it with tighter priors; `ScaleRefinement()` (`1430`) refines scale only. `mbBadImu` (`144`) signals a failed init back to Tracking.

### Correction (loop closing / merging and GBA)

`LoopClosing::Run()` (`src/LoopClosing.cc:90`) calls `NewDetectCommonRegions()` (`324`). It first tries to *continue* an in-progress candidate via `DetectAndReffineSim3FromLastKF` (`535`), propagating the last Sim3 and re-optimizing; a candidate is confirmed only after `>=3` temporally-consistent coincidences (`396`,`444`,`880`). Otherwise `DetectNBestCandidates` queries DBoW2 and `DetectCommonRegionsFromBoW` (`578`) runs `SearchByBoW`, a `Sim3Solver` RANSAC, coarse+fine `SearchByProjection`, `OptimizeSim3` (`767`), and a covisibility geometric vote (`nNumKFs>=3`).

`CorrectLoop` (`969`) stops LocalMapping, aborts any running GBA (`mnFullBAIdx++`, `985`), propagates the loop Sim3 to covisible KFs and their map points, fuses matched loop points, runs `OptimizeEssentialGraph` (or the `…4DoF` inertial variant), then spawns `RunGlobalBundleAdjustment`. Multi-map merges use `MergeLocal` (`1215`, visual welding window) or `MergeLocal2` (`1783`, inertial `ApplyScaledRotation` + `MergeInertialBA`). `RunGlobalBundleAdjustment` (`2268`) runs full BA then propagates the result down the spanning tree; staleness is detected by capturing `idx=mnFullBAIdx` (`2300`) and comparing under `mMutexGBA`.

### The IMU / BA maths

Every optimizer wraps g2o. `PoseOptimization` (`Optimizer.cc:814`) is motion-only BA over a single `VertexSE3Expmap` with landmarks fixed inside each reprojection edge; four LM rounds re-classify inliers/outliers against chi2 (5.991/7.815), and at `it==2` the robust kernel is removed so the final round is plain weighted least squares. Reprojection edges use information `mvInvLevelSigma2[octave]·I` (divided additionally by `mpCamera->uncertainty2` for fisheye); inertial edges (`EdgeInertial`/`EdgeInertialGS`, `EdgeGyroRW`/`EdgeAccRW`) take information from the inverted preintegration covariance blocks with Huber `δ=sqrt(16.92)` (`2699`). Gauge is fixed by pinning the origin/init KF (LocalBA `1277`, essential graph `1614`), all poses in VI-init `InertialOptimization` (`3129`), and `pLoopKF` in the 4DoF graph (`5408`). Scale/gravity are handled by `VertexScale`+`VertexGDir` with `EdgeInertialGS` (scale free only for monocular, `3180`). `OptimizeEssentialGraph` (`1558`) is a Strasdat-style Sim3 pose graph [R18]; the 4DoF variant (`5349`) now correctly sets `matLambda(0,0)=(1,1)=(2,2)=1e3`. `LocalInertialBA`/`MergeInertialBA`/`FullInertialBA` build sliding temporal windows over `mPrevKF`, marginalize points, and downweight the window-boundary inertial edge (`info*1e-2`).

The **new covariance block** (`Optimizer.cc:1113-1168`) forms `H = Σ Jᵀ Ω J` directly over inlier motion-only edges using the analytic pose Jacobians (`jacobianOplusXi`, column order `[ω;υ]` matching `SE3Quat::exp`), symmetrizes and eigen-decomposes it, and — gated on `>=6` inlier edges, solver success, and `λ_min > 1e-9·max(λ_max,1)` — stores `Σ = V diag(1/λ) Vᵀ = H^-1` into `Frame::mPoseCovariance`.

### The ROS 2 integration

The wrapper is `OrbSlam3LifecycleNode` (`rclcpp_lifecycle::LifecycleNode`). `on_configure` (`orb_slam3_node.cpp:115`) reads params and constructs the `ORB_SLAM3::System` (which spawns LocalMapping/LoopClosing threads) plus four lifecycle publishers, a TF broadcaster, and a diagnostic_updater; `on_activate` activates publishers and calls the subclass `createSubscriptions()`; `on_cleanup` `Shutdown()`s the System. Each sensor node subclasses it (mono = plain image sub; stereo/rgbd/stereo_inertial = `message_filters` `ApproximateTime` sync plus an `Imu` sub). Inertial callbacks call `drainImu(stamp)` (`291`) to hand ORB-SLAM3 the buffered IMU with `t<=frame-time`, invoke `Track*`, then `publishTracking(Tcw, stamp)` (`303`), which inverts `Tcw→Twc` and broadcasts TF `map→camera` plus PoseStamped/Odometry/Path/PointCloud2. Odometry covariance comes from `computePoseCovariance` (`426`): `kStatic` diagonal, `kQuality` (scaled by `(ref/inliers)²`), or `kG2o`, which pulls the motion-only marginal via `GetTrackedPoseCovariance` and maps the left-perturbation-of-`Tcw` covariance through the SE3 adjoint of `Twc` into the ROS `[trans,rot]` world frame. `mf_compat.hpp`/`distro_compat.hpp` shim message_filters and cv_bridge differences across Humble/Jazzy/Lyrical.

### Build and packaging

The package rebuilds ORB-SLAM3 as an `ament_cmake_auto` library plus a ROS-node component library in one colcon build. `CMakeLists.txt` is a hybrid: ament tooling creates the target (`ament_auto_add_library`, `121`) while non-ament system deps (OpenCV, Eigen3, Boost::serialization, OpenSSL, optional Pangolin) are found and linked by hand (`64-74`,`146-157`). Vendored `Thirdparty/DBoW2` and `g2o` are compiled in-tree as SHARED; Sophus is header-only. `USE_PANGOLIN` is a PRIVATE compile definition with a forward-declared `pangolin::OpenGlMatrix` and `#ifdef` guards, so `USE_PANGOLIN=OFF` yields a genuine headless build. Because `ament_target_dependencies` was removed in ament_cmake 2.8, ROS-node deps are linked by iterating each package's `${dep}_TARGETS`.

## Findings

### High

#### H1. `PreintegrateIMU` early returns still leave `mpImuPreintegratedFrame` NULL, so the common `PredictStateIMU` branch NULL-derefs (audit fix #9 incomplete)
One-line: A VI frame with 0–1 IMU samples segfaults in the majority `!mbMapUpdated` path.
**File** `src/Tracking.cc:1775`. The applied fix for audit item #9 set only `mCurrentFrame.mpImuPreintegrated` on the empty-queue (`1639`) and `n==0` (`1683`) early returns, but left `mpImuPreintegratedFrame` NULL (Frame ctor inits it NULL, `Frame.cc:104`). `PredictStateIMU`'s common branch (`1769`) dereferences `mpImuPreintegratedFrame->dT` (`1775`) and `->GetDeltaRotation/Position/Velocity` (`1777-1779`) with no guard; since `mbMapUpdated` is false on every frame where no BA/loop finished, the exact deref #9 claimed to fix is still live. The same unset pointers also endanger `StereoInitialization` (`2344`) and `UpdateFrameIMU` (`4019`).
**Trigger** IMU_MONOCULAR/STEREO/RGBD run where a camera frame carries 0 or 1 usable IMU samples (IMU≈cam rate, dropped packet, or cam/IMU timeshift, issue #730); the `n==0` branch runs, `mbMapUpdated` is false → NULL deref → segfault.
**Fix** In both early-return branches also build a valid (possibly empty) `mpImuPreintegratedFrame` from `mLastFrame.mImuBias` and set `mCurrentFrame.mpLastKeyFrame`; or add explicit NULL guards at `1775`, `2344`, `4019` that fall back to the visual model. [R4][R3][R5]

#### H2. `InitializeIMU`/`ScaleRefinement` `delete` queued KeyFrames that Tracking still owns → use-after-free
One-line: During VIBA1/VIBA2 the guard that suppresses new KFs is bypassed, and a KF Tracking still points to is `delete`d.
**File** `src/LocalMapping.cc:1415`. After the final KF drain and the long GBA-correction loops, `InitializeIMU` (`1415-1420`) and `ScaleRefinement` (`1484-1489`) do `SetBadFlag(); delete *lit;` over `mlNewKeyFrames`. `Tracking::CreateNewKeyFrame` is only suppressed while the IMU is *not yet* initialized (`Tracking.cc:3215`); during VIBA1/VIBA2 the map is already IMU-initialized, so Tracking keeps creating KFs — one inserted after the last drain sits in `mlNewKeyFrames`, is Tracking's live `mpLastKeyFrame`, is linked into the temporal chain, and (IMU_STEREO/RGBD) holds MapPoint observations. Deleting it dangles those pointers, violating the documented flag-bad-never-free invariant. *Verification was mixed (1 confirm / 1 refute) — confidence medium; the window is real but narrow.*
**Trigger** IMU session reaching VIBA1/VIBA2 after `isImuInitialized()`; Tracking creates a KF during the correction loops, which is then `delete`d; the next Tracking frame dereferences the freed `mpLastKeyFrame`.
**Fix** Do not `delete` queued KeyFrames — drain them through `ProcessNewKeyFrame()`, leave them for the next `Run()` pass, or at most `SetBadFlag()` and let normal culling reclaim them. [R6][R1][R2]

#### H3. `DetectCommonRegionsFromBoW` passes member `mbFixScale` to `OptimizeSim3` instead of the local `bFixedScale`, freezing scale for VI-monocular place recognition before BA2
One-line: A copy-paste defect bakes scale=1.0 into loop/merge Sim3 while a monocular-inertial map is still scale-ambiguous.
**File** `src/LoopClosing.cc:767`. Lines `763-765` compute `bFixedScale=mbFixScale` then override it to `false` for IMU_MONOCULAR maps with `GetIniertialBA2()==false`, but line `767` passes `mbFixScale` (true for all IMU sensors) to `OptimizeSim3`. Scale is forced fixed while the map is scale-free, biasing `gScm`/`mg2oLoopSlw`/`mg2oMergeSlw` and the essential-graph correction. Sibling sites (`556`,`1183`) use the corrected variable, confirming the defect.
**Trigger** IMU_MONOCULAR session, a BoW loop/merge candidate validated after `isImuInitialized()` but before `GetIniertialBA2()`.
**Fix** Pass the local override: `OptimizeSim3(..., bFixedScale, mHessian7x7, true)`. [R1]

#### H4. `RunGlobalBundleAdjustment` early-return on concurrent IMU init leaks `mbRunningGBA=true`, hanging `System::Shutdown()` forever
One-line: A visual GBA discarded when IMU init completes never clears its run flags, so the restored Shutdown wait spins forever.
**File** `src/LoopClosing.cc:2313`. The guard `if(!bImuInit && pActiveMap->isImuInitialized()) return;` (`2312-2313`) correctly discards the stale visual result but returns *before* the only flag-reset site (`mbFinishedGBA=true; mbRunningGBA=false` at `2508-2509`). Nothing else clears `mbRunningGBA`. This fork restored the Shutdown wait (`System.cc:556`), so the leak becomes a hard hang, and every later `CorrectLoop`/`MergeLocal` sees `isRunningGBA()==true`.
**Trigger** IMU session, a loop/merge launches a visual GBA before IMU init, and `LocalMapping::InitializeIMU` sets the map IMU-initialized during the BA.
**Fix** Set `mbFinishedGBA=true; mbRunningGBA=false;` before each early return at `2310`/`2313`, or use a scope guard that clears the flags on any exit while holding `mMutexGBA`. [R2][R6]

#### H5. Relocalization `SearchByProjection` reads `pKF->mvKeysUn[i]` out of bounds for fisheye right-camera map points
One-line: The rotation-consistency histogram indexes a `NLeft`-sized vector with a full `N`-range index on stereo-fisheye.
**File** `src/ORBmatcher.cc:1973`. The loop iterates `i` over `pKF->GetMapPointMatches()` (size `N=NLeft+NRight`), then reads `pKF->mvKeysUn[i].angle`. For fisheye, `mvKeysUn.size()==NLeft` (`Frame.cc:750-753`; copied at `KeyFrame.cc:51`). Any right-only match (`i>=NLeft`) indexes past the end → UB (garbage bin or segfault). The sibling BoW matcher guards this exact case (`800`,`820`).
**Trigger** Stereo-fisheye (TUM-VI) relocalization where a candidate KF holds a MapPoint at a right-only slot `i∈[NLeft,N)` that reprojects and matches with orientation check on.
**Fix** Skip `i` when `pKF->NLeft!=-1 && i>=pKF->mvKeysUn.size()`, or select the keypoint with the standard left/right ternary. [R1][R5]

#### H6. `KeyFrameDatabase::DetectNBestCandidates` infinite-loops on a bad keyframe (`continue` without advancing the iterator)
One-line: A culled covisible KF in the accumulated matches hangs the Loop-Closing thread.
**File** `src/KeyFrameDatabase.cc:712`. The selection loop advances `i`/`it` only at the tail (`727-728`), but `if(pKFi->isBad()) continue;` (`712-713`) jumps over both increments. Covisibility neighbours pushed at `678` are not `isBad`-filtered, and `isBad()` is sticky (never freed), so the loop spins forever whenever the loop still "wants" more candidates. *Confidence medium.*
**Trigger** `NewDetectLoop → DetectNBestCandidates` with a recently-culled covisible KF present in `lAccScoreAndMatch`; e.g. loop-only recognition keeps `vpMergeCand.size()<nNumCandidates` true.
**Fix** Advance before continuing: `if(pKFi->isBad()){ i++; it++; continue; }`. [R2][R24]

#### H7. ROS node broadcasts `map→camera` TF directly, giving `camera` two parents (REP-105 violation)
One-line: With the shipped `base_link→camera` static TF, the camera frame is double-parented and the tf2 tree breaks. *(Merged: two findings — `orb_slam3_node.cpp:336` High and `orb_slam3_node.cpp:337` Medium — describe the same defect.)*
**File** `orb_slam3_ros/src/orb_slam3_node.cpp:336`. `publishTracking()` broadcasts `frame_id=world_frame_id_('map')`, `child_frame_id=camera_frame_id_('camera')` (`336-338`). The package's `base_to_camera.launch.py` publishes `base_link→camera`, so with both running `camera` has two parents → `TF_MULTIPLE_PARENT`, inconsistent lookups, oscillating tree. A SLAM source must publish `map→odom` (or `map→base_link`) per REP-105, never a transform whose child already has a parent. `odom_child_frame_id_` only relabels the `~/odom` message (`392`), not the TF.
**Trigger** Run any node with defaults plus `base_to_camera.launch.py`, or any robot whose camera frame is already a URDF child of `base_link`.
**Fix** Publish `map→odom` by composing with the incoming `odom→base` transform (factor out `base→camera`), or make the parent/child configurable and document that `camera_frame_id` must be a SLAM-private leaf frame. [R19]

#### H8. `LocalInertialBA` force-stop fix dropped upstream's early-return guard, so an aborted BA culls observations on UN-optimized residuals
One-line: The applied `setForceStopFlag` fix replaced (not augmented) upstream's abort guard, so a 0-iteration aborted run permanently thins the local map on pre-optimization chi2.
**File** `src/Optimizer.cc:2898`. The fix added `setForceStopFlag(pbStopFlag)` before `optimize()` (`2898-2902`) but removed upstream's `if(pbStopFlag) if(*pbStopFlag) return;` guard, and there is no post-optimize `if(*pbStopFlag) return;` before the outlier-erase loop (`2956-2965`). If `*pbStopFlag` is already set, g2o runs zero iterations, `err==err_end` so the FAIL guard at `2948` does not fire, and observations are erased on pre-optimization chi2 (`2919`/`2936`). `LocalBundleAdjustment` guards its culling with `bDoMore=false` (`1463-1464`); `LocalInertialBA` does not.
**Trigger** Inertial mode: while `LocalInertialBA` builds its graph, Tracking's `NeedNewKeyFrame` calls `InterruptBA()` (`Tracking.cc:3194`), presetting the flag; `optimize()` runs 0 iterations and the code culls observations.
**Fix** Restore `if(pbStopFlag && *pbStopFlag) return;` before `initializeOptimization()` and/or between `optimize()` and the erase loop, while keeping `setForceStopFlag` for genuine mid-run abort. [R24][R17]

### Medium

#### M1. `UpdateLocalKeyFrames` temporal loop advances the cursor only inside the not-yet-added branch, so it adds zero temporal KFs in the normal case
One-line: The VI local-BA window is silently starved of its temporally-adjacent keyframes.
**File** `src/Tracking.cc:3590`. The cursor advance `tempKeyFrame = tempKeyFrame->mPrevKF;` (`3594`) is inside the `if(mnTrackReferenceForFrame != mnId)` block; the immediate last KF was almost always already added by the covisibility loop, so the `if` is false on iteration 0, the cursor never advances, and all 20 iterations spin on the same KF.
**Trigger** Any steady-state inertial frame whose last KF shares points with the current frame (the normal case).
**Fix** Move `tempKeyFrame = tempKeyFrame->mPrevKF;` out of the `if` so it runs every iteration. [R1]

#### M2. `mbBadImu` internal-reset path is a cross-thread data race and can drop the map reset (`mpMapToReset` is a dead store)
One-line: A non-atomic bool read unsynchronized every frame, plus a reset that may never clear the atlas.
**File** `src/LocalMapping.cc:144`. `mbBadImu` (plain bool, `LocalMapping.h:106`) is written under `mMutexReset` but read with no lock at `Tracking.cc:1810` (data race, UB). The actual atlas clear is only done by `Tracking::ResetActiveMap`; `LocalMapping::ResetIfRequested` (`1126-1141`) clears `mbBadImu` (`1136`) without touching the atlas and never reads `mpMapToReset` (a dead field). If the reset clears the flag before Tracking observes it, the failed-init map is never reset. *Confidence medium.*
**Trigger** `Run()` sets `mbBadImu` at `144`; `ResetIfRequested` clears it ~tens of ms later; if no Tracking frame lands in that window (or the store is reordered/invisible), `ResetActiveMap` is never called.
**Fix** Make `mbBadImu` `std::atomic<bool>` (or guard it), route the internal reset through the same `RequestResetActiveMap(pMap)` API Tracking uses, and honor or remove `mpMapToReset`. [R7][R1][R5]

#### M3. `MergeLocal2` never clears member `mvpMergeConnectedKFs`, so from the second merge on it fuses against stale first-merge keyframes
One-line: A member vector is truncated-to-6 but never cleared, so new merge KFs land past index 6 and are erased.
**File** `src/LoopClosing.cc:1961`. `mvpMergeConnectedKFs` (`LoopClosing.h:208`) is `push_back`ed and then truncated to the first 6 entries (`1964-1965`) but never cleared; on later merges the newest entries land at index `>=6` and are erased, leaving the first-merge KFs to drive `spMapPointMerge`/`SearchAndFuse` (`1979-1985`).
**Trigger** Any inertial session performing two or more successful `MergeLocal2` merges.
**Fix** `mvpMergeConnectedKFs.clear();` at the start of `MergeLocal2`, or make it a local vector as in `MergeLocal`. [R1]

#### M4. `RunGlobalBundleAdjustment` staleness guard can be defeated: `mnFullBAIdx` read outside `mMutexGBA`, `mbStopGBA` reset by a concurrent relaunch
One-line: A stale GBA can overwrite a freshly corrected map because both its guard tokens can be reset by the relaunch.
**File** `src/LoopClosing.cc:2300`. `idx=mnFullBAIdx` is read without the lock (`2300`); because the aborting increment happens before the old thread reaches `2300`, `if(idx!=mnFullBAIdx) return;` (`2309`) does not fire, and the relaunch resets `mbStopGBA=false` (`1203`/`1768`) without the lock, so the superseded thread can write stale `mTcwGBA`/`mPosGBA` into the corrected map. *Confidence medium.*
**Trigger** A loop/merge that aborts a running GBA and immediately launches a new one; the aborted thread finishes `optimize()` after `mbStopGBA` is reset.
**Fix** Capture `idx` under `mMutexGBA` at BA launch and compare that token; keep `mbStopGBA` writes under the lock; gate the update section on `(idx==mnFullBAIdx && !mbStopGBA)` atomically. [R2]

#### M5. Motion-only marginal covariance treats map points as noise-free, so `mPoseCovariance` is systematically over-confident
One-line: `H^-1` is the *conditional* (map-fixed) covariance, not the marginal; the "exact" comment is misleading. *(Merged: `Optimizer.cc:1131` and `Optimizer.cc:1159` are the same finding; see the covariance assessment below.)*
**File** `src/Optimizer.cc:1131` (`:1159`). `H=Σ Jᵀ Ω J` is accumulated with landmarks fixed (`1124-1148`); the true marginal requires the Schur term `H_pp − H_pl H_ll^-1 H_lp`, which subtracts a PSD matrix and yields a *larger* covariance. Because ORB map points (low-parallax/distant) carry real position uncertainty, `Σ` under-estimates true pose uncertainty; the comment at `1116-1118`/`1117` calling it "exact … no Schur marginalization is needed" is true only for the conditional. Consumed unmodified at `System.cc:331-333` and published on `~/odom`, where an EKF (robot_localization) over-trusts VO. *One confirm / one refute in verification — the math claim is solid; the practical severity is scene-dependent.*
**Trigger** A frame whose inliers are mostly far/low-parallax points passes the count/conditioning gates; `trace(Σ)` can be an order of magnitude too small; an EKF locks onto a wrong pose.
**Fix** Document/rename the field as a conditional pose information and inflate downstream (default `covariance_g2o_scale > 1`), or recover the true marginal via the Schur complement. At minimum correct the "exact" comment. [R10][R11][R12][R13][R1]

#### M6. No pose covariance is ever produced in inertial tracking despite the 6×6 pose information already being assembled
One-line: After IMU init the covariance feature silently never fires, though the inertial optimizers already build the pose Hessian.
**File** `src/Tracking.cc:2987`. Once `isImuInitialized()`, `TrackLocalMap` routes to `PoseInertialOptimizationLast{Frame,KeyFrame}` (`2982`/`2987`), neither of which sets `mPoseCovariance`/`mbHasPoseCovariance`; every frame defaults `false` (`Frame.h:200`), so `mbTrackedPoseCovarianceValid=false` permanently in VI mode. Yet `PoseInertialOptimizationLastKeyFrame` already builds `H.block<6,6>(0,0)` (`Optimizer.cc:4889-4925`), stored only in `ConstraintPoseImu`.
**Trigger** Any EuRoC/TUM-VI IMU run after IMU init: `mTrackedPoseCovariance` is invalid on every tracked frame.
**Fix** Invert the (Schur-reduced) `H.block<6,6>(0,0)` with the same conditioning gate and populate `mPoseCovariance`/`mbHasPoseCovariance` in the inertial optimizers. [R1][R3]

#### M7. `OptimizeSim3` weights synthetic out-of-KF2 observations at octave 0 (max information) via `cv::KeyPoint` constructor misuse
One-line: `mnTrackScaleLevel` is passed to the `_size` slot, so `octave` stays 0 and the least-certain synthetic matches get the largest weight.
**File** `src/Optimizer.cc:2337`. For `i2<0` (bAllPoints) the code builds `cv::KeyPoint(Point2f, pMP2->mnTrackScaleLevel)`; the third positional arg is `_size`, not `_octave`, so `kpUn2.octave==0` and `invSigmaSquare2 = mvInvLevelSigma2[0] = 1.0` (`2348`). Both LoopClosing callers pass `bAllPoints=true` (`556`,`767`).
**Trigger** Every loop/merge Sim3 refinement where a matched point is not observed in KF2.
**Fix** Set the octave explicitly, e.g. `cv::KeyPoint(Point2f(x,y), 1.f, -1, 0, pMP2->mnTrackScaleLevel)` or `kpUn2.octave = pMP2->mnTrackScaleLevel`. [R14][R2][R18]

#### M8. `MergeInertialBA` gives covisibility KFs free velocity/bias vertices that no inertial edge constrains
One-line: Isolated free inertial variables contradict the routine's own "not optimized" comment; the window boundary is also unanchored.
**File** `src/Optimizer.cc:4201`. The comment at `4013` says cov-KF inertial params are not optimized, but the cov-KF loop adds `VertexVelocity`/`VertexGyroBias`/`VertexAccBias` with `setFixed(false)` (`4201-4211`); inertial edges are created only for `vpOptimizableKFs` (`4246`), so cov-KF velocity/bias are free but touched by no edge. The temporal anchor is pushed with free velocity/bias (`4035`) instead of fixed as in `LocalInertialBA` (`2635-2643`). *Confidence medium.*
**Trigger** Any inertial map merge routed through `MergeInertialBA`.
**Fix** `setFixed(true)` the cov-KFs' VV/VG/VA and fix the temporal-boundary KF's velocity/bias, matching `LocalInertialBA`. [R1]

#### M9. `Fuse(bRight)` indexes `mvuRight` (size `NLeft`) with a local right-camera index in `[0,NRight)`
One-line: A right-camera disparity read happens before the `+NLeft` normalization, OOB when `NRight>NLeft`.
**File** `src/ORBmatcher.cc:1272`. `GetFeaturesInArea(...,bRight=true)` returns local indices into `mvKeysRight` (`[0,NRight)`); line `1272` reads `pKF->mvuRight[idx]` before the `idx += pKF->NLeft` normalization at `1298`, but `mvuRight.size()==NLeft` for fisheye. When in bounds it reads a *left* keypoint's flag (semantically wrong; harmless only because fisheye `mvuRight` is uniformly −1); when `NRight>NLeft` it is OOB. `SearchForTriangulation` (`979`,`1007`) guards the identical access.
**Trigger** Fisheye keyframe with `NRight>NLeft` where a fused map point falls near a high-index right keypoint. Called for every fisheye KF (`LocalMapping.cc:774`,`804`).
**Fix** `if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)`, or index `mvuRight` only after the `+NLeft` normalization with a size check. [R1]

#### M10. `SearchBySim3` (and the `vpPointsKFs` `SearchByProjection` overload) project with a hardcoded pinhole model even for KannalaBrandt fisheye
One-line: `u=fx·x+cx` is the wrong projection for fisheye, silently losing loop/merge correspondences.
**File** `src/ORBmatcher.cc:1518`. Both directions (`1518-1519`, `1594-1599`) and the `vpPointsKFs` overload (`577`) use pinhole projection; for a KB8 keyframe peripheral points fold or shift by many pixels, so `GetFeaturesInArea` is queried at the wrong pixel. Sibling Sim3 overloads correctly use `pKF->mpCamera->project` (`465`,`1379`). Confirmed identical in upstream master.
**Trigger** Loop/merge between two fisheye keyframes where an off-axis map point misses its keypoint.
**Fix** Replace the manual pinhole projection with `pKF->mpCamera->project(p3Dc)` / `pKF2->mpCamera->project(...)`. [R15][R1][R24]

#### M11. `KannalaBrandt8::projectJac` produces NaN (0/0) on the optical axis, poisoning the g2o Hessian
One-line: Division by `r2`/`r3` at the principal point gives `NaN`, which propagates into the BA step.
**File** `src/CameraModels/KannalaBrandt8.cpp:163`. `projectJac` divides by `r2=x²+y²` and `r3=r2·r` with no guard; at `x=y=0` (and `f=θ=0`) `JacGood(0,0)=fx·(0/0+0/0)=NaN`. One NaN entry destroys the LM step. The pinhole `projectJac` has no such singularity. Flagged in audit item #12.
**Trigger** A landmark reprojecting onto/within a fraction of a pixel of the fisheye principal point during `LocalBundleAdjustment`.
**Fix** Guard `if(r2 < 1e-12)` and return the finite small-angle limit (diagonal `fx/z`, `fy/z`, zero cross terms), or clamp `r2` to a small positive floor. [R15][R5]

#### M12. `KannalaBrandt8::unproject` clamps the normalized image radius to π/2 and forces `z=+1`, mis-unprojecting wide-FOV (>90°) features
One-line: `theta_d` is a radius, not an angle; the `fminf(...,π/2)` caps it and `fmaxf(-π/2,...)` is dead code, and forcing `z=+1` flips rays past 90°.
**File** `src/CameraModels/KannalaBrandt8.cpp:121`. Line `120` computes the normalized radius; line `121` caps it at π/2; line `139` builds `scale=tan(θ)/theta_d`; line `142` returns `z=1`. For incidence angles near/above 90° the true ray has `z<0`, so `project(unproject(p)) != p` for border features. Flagged in audit item #11. *Confidence medium.*
**Trigger** TUM-VI ~195° fisheye border keypoints (half-angle ~97.5°).
**Fix** Remove the meaningless `fmaxf(-π/2,...)`, do not clamp the radius, optionally reject features beyond model FOV, and return a true unit ray so `θ>π/2` yields correct negative-`z`. [R15][R1]

#### M13. `CheckRT` ignores `GeometricTools::Triangulate`'s failure return, reading an uninitialized 3D point
One-line: On the `x3Dh(3)==0` early-false path `p3dC1` stays uninitialized and a garbage-finite point can be counted in `nGood`.
**File** `src/TwoViewReconstruction.cc:832`. `p3dC1` is uninitialized (`828`); `Triangulate` (`832`) discards its bool return; that function returns false without writing `x3D` when the homogeneous 4th coord is zero (`GeometricTools.cc:59-60`). Line `835` then tests `isfinite` on stack garbage, and a bogus point can enter `vP3D` (`883`) during monocular init. A refactor regression vs the original always-write triangulator.
**Trigger** A low-parallax / near-pure-rotation match whose DLT null vector has `x3Dh(3)==0`.
**Fix** Check the return: `if(!Triangulate(...)){ vbGood[...]=false; continue; }`, or init `p3dC1` to NaN. [R10]

#### M14. `Settings::readCamera2` reads the second fisheye camera's distortion from `Camera1.k1..k4`
One-line: Camera2 gets Camera1's radial polynomial, mis-calibrating the right lens of any dissimilar stereo-fisheye rig.
**File** `src/Settings.cc:318`. `fx/fy/cx/cy` come from `Camera2.*` (`313-316`) but `k0..k3` are read from `Camera1.k1..k4` (`318-321`). This corrupts every right-image project/unproject, `ComputeStereoFishEyeMatches`, epipolar checks, and right-camera reprojection edges. Confirmed identical in upstream master.
**Trigger** Any STEREO/IMU_STEREO KannalaBrandt config (File.version 1.0) whose Camera2 distortion differs from Camera1 (e.g. TUM-VI).
**Fix** Read from `Camera2.k1..k4`. [R24][R15][R1]

#### M15. `Map::PreSave` erases the current MapPoint from `mspMapPoints` mid range-for, invalidating the loop iterator (UB on atlas save)
One-line: `EraseObservation` can drop a point below 2 observations → `SetBadFlag` → `mspMapPoints.erase` on the element being iterated.
**File** `src/Map.cc:362`. The range-for over `mspMapPoints` (`362`) calls `EraseObservation` (`376`), which on `nObs<=2` calls `SetBadFlag → Map::EraseMapPoint → mspMapPoints.erase(pMP)` (`Map.cc:101`), invalidating the range-for's cached iterator; the subsequent `++` is UB. Confirmed identical in upstream master. *Confidence medium.*
**Trigger** Saving an atlas whose map holds points with cross-map/bad-KF observations (post-merge residue) that drop below the 2-observation threshold.
**Fix** Snapshot the set into a `vector` and iterate the copy, or collect points to bad-flag and erase after the loop. [R7][R24]

#### M16. `Atlas::SetMapBad` / `RemoveBadMaps` mutate `mspMaps`/`mspBadMaps` without holding `mMutexAtlas`
One-line: A merge on the Loop-Closing thread mutates the atlas map set during concurrent locked traversals.
**File** `src/Atlas.cc:260`. Every other container accessor takes `mMutexAtlas` (`GetAllMaps 209-222`, `CountMaps 224-228`, `CreateNewMap 60`), but `SetMapBad` (`260-266`) and `RemoveBadMaps` (`268-276`) do not, racing `std::set` mutation against locked readers — iterator invalidation, not merely a torn scalar. *Confidence medium.*
**Trigger** `LoopClosing` merge `SetMapBad(pCurrentMap)` (`LoopClosing.cc:1551`) racing a concurrent `GetAllMaps`/`CountMaps`/`CreateNewMap`.
**Fix** Take `unique_lock<mutex> lock(mMutexAtlas)` in both methods. [R6][R1]

#### M17. IMU buffer has no size cap; a persistent IMU/image clock offset makes `drainImu` never drain
One-line: `pushImu` appends unbounded; if IMU stamps run ahead of image stamps the deque grows forever.
**File** `orb_slam3_ros/src/orb_slam3_node.cpp:288`. `pushImu` (`287-288`) appends with no capacity check; `drainImu(t_frame)` (`291-301`) pops only while `front().t <= t_frame`. A persistent offset (hardware-vs-ROS-time, `use_sim_time`, Kalibr `t_imu=t_cam−offset`) drains nothing → monotonic memory growth and latency. The `~/path` buffer was capped but the IMU deque was not. *Verification neutral (0/0) — plausible, not confirmed.*
**Trigger** IMU stamps offset ahead of image stamps, or images stall while IMU streams.
**Fix** Cap `imu_buffer_` by max count or max age relative to the newest stamp, with a throttled warning on drop. [R4]

#### M18. Path/pose/TF are published under a single fixed world frame across Atlas map switches and resets
One-line: New-map creation teleports `map→camera`; `path_` then mixes poses from two unrelated coordinate frames.
**File** `orb_slam3_ros/src/orb_slam3_node.cpp:328`. `publishTracking` emits `Twc` under `world_frame_id_` whenever state==OK and accumulates every pose into `path_` (`370`), with no notion of the active Atlas map id and no reset on map change/merge. After a new map the TF jumps to the new origin; after a loop/merge the already-published TF/path are stale.
**Trigger** Tracking lost long enough for a second Atlas map, or a loop closure/merge correction.
**Fix** Expose the active map id via `System`; on change clear `path_` and publish a distinct frame id / discontinuity marker; on merge republish the corrected path. [R1]

#### M19. In pure monocular the g2o covariance is in arbitrary map-scale units, published as if metric
One-line: The translation covariance is `(map-unit)²`, and the lever-arm term grows with `|twc|²` in map units.
**File** `orb_slam3_ros/src/orb_slam3_node.cpp:451`. `Ad·Σ·Adᵀ` and the block reorder are dimensionally consistent, but for MONOCULAR `Σ_cw` is in the map's unobservable gauge scale; written verbatim to `odom.pose.covariance` (`394`,`459`), its magnitude is meaningless relative to metric sensors and drifts as scale drifts. *One confirm / one refute — the scale argument is correct; whether it matters depends on the fusion setup.*
**Trigger** Monocular run where ORB-SLAM3 rescales after IMU/loop events, fused against a metric wheel/IMU source.
**Fix** Gate g2o covariance mode to metric-scale sensors (stereo/RGBD or post-IMU-init), or normalize by current map scale; document monocular `~/odom` covariance as non-metric. [R2][R1]

#### M20. Covariance is discontinuous when g2o mode falls back to the quality diagonal (different magnitude and structure)
One-line: Per-frame switching between a dense g2o marginal and an axis-aligned quality diagonal jumps 1–3 orders of magnitude.
**File** `orb_slam3_ros/src/orb_slam3_node.cpp:464`. Valid frames publish a dense correlated 6×6 (`433`); invalid frames (conditioning fails, or inertial frames with no `mPoseCovariance`) fall to a `pose_cov_diagonal_·(ref/inliers)²` diagonal (`464-484`) with no reconciliation, which a consistency-sensitive filter reads as sudden de/re-trust.
**Trigger** Borderline-observable scenes where `mbHasPoseCovariance` toggles frame-to-frame (`Optimizer.cc:1154` near threshold), or stereo-inertial where the visual optimizer is not last.
**Fix** On fallback, scale the diagonal to the recent valid g2o magnitude, or hold the last valid marginal with mild time-based inflation. [R20]

#### M21. `System::Shutdown` restored the mapper/loop wait but never stops/joins the Pangolin viewer thread → use-after-free on teardown
One-line: The viewer half of Shutdown is still commented out and no thread is ever joined, so a viewer-enabled build reads freed objects after Shutdown returns.
**File** `src/System.cc:556`. The wait loop (`556-559`) correctly blocks on LocalMapping/LoopClosing/GBA, but `mpViewer->RequestFinish()`+wait remains commented (`544-549`) and none of `mptViewer`/`mptLocalMapping`/`mptLoopClosing` (raw `new std::thread`, `199`/`216`/`236`) are ever joined. With `USE_PANGOLIN=ON` and `use_viewer=true`, the viewer thread outlives Shutdown and the subsequent `System` destruction (e.g. `on_cleanup` → `slam_.reset()`). The wait is also unbounded. ROS default `use_pangolin_viewer=false` masks it. *Confidence medium.*
**Trigger** Viewer-enabled build; standalone examples or any config that starts the viewer.
**Fix** Re-enable the viewer `RequestFinish()`+wait (or add it to the predicate), join/detach the three threads in `~System`, and optionally bound the wait with a timeout. [R2]

### Low

#### L1. `PredictStateIMU()` return value ignored on RECENTLY_LOST and motion-model IMU paths
**File** `src/Tracking.cc:1994`. The inertial branch presets `bOK=true` and discards the bool (`1994`); `TrackWithMotionModel` calls it then `return true` (`2862-2863`). `PredictStateIMU` returns false without setting a pose when `!mpPrevFrame` (`1745`) or in its final else (`1787`), so a failed prediction still reports OK and publishes a stale/unset pose. *Confidence medium; verification neutral (0/0).*
**Trigger** Inertial frame where `mbMapUpdated` is true but `mpLastKeyFrame` is momentarily NULL (post map-change), or a frame lacking `mpPrevFrame`, during RECENTLY_LOST recovery.
**Fix** `bOK = PredictStateIMU();` at `1994` and `return PredictStateIMU();` at `2862-2863`. [R1]

#### L2. Gravity-alignment rotation in `InitializeIMU` divides by `|gI × dirG|` with no degeneracy guard → NaN/ill-conditioned `Rwg` for near-vertical gravity
**File** `src/LocalMapping.cc:1251`. `vzg = v*ang/nv` (`1251`) with `nv=|gI×dirG|` unguarded; a down/up-facing first keyframe makes `dirG≈(0,0,±1)` so `nv→0` and `v*ang/nv` is 0/0 or noise-driven, then `exp(vzg)` corrupts the whole map via `ApplyScaledRotation` at init. Same class as audit #30's `LogSO3` singularity. *Confidence medium.*
**Trigger** Any rig whose first keyframe orientation aligns `dirG` with gravity.
**Fix** If `nv<eps`, set `Rwg=I` when `cosg>0` or a π rotation about any axis ⊥ `gI` when `cosg<0`; otherwise the current formula. [R9][R1][R8]

#### L3. `CreateNewMapPoints` reads `mpTracker->mState` across threads without synchronization
**File** `src/LocalMapping.cc:465`. The `bCoarse` gate reads `mpTracker->mState` (written on the Tracking thread) from Local Mapping with no lock — a data race that degrades matching decisions rather than crashing. *Confidence medium.*
**Trigger** Concurrent tracker state transition while `CreateNewMapPoints` evaluates `bCoarse`.
**Fix** Expose tracking state via a synchronized accessor (mutex or `std::atomic<int>`) and snapshot once. [R7][R6]

#### L4. `MergeLocal` inertial welding-window walk reads `mpCurrentKF->mPrevKF/mNextKF` instead of `pKFi->…` (latent: guarded by an always-false condition)
**File** `src/LoopClosing.cc:1284`. Lines `1284`/`1299`/`1360` advance with the fixed `mpCurrentKF`, so the window never advances (infinite insert) or null-derefs `GetMapPoints()` (`1287`); the merge side (`1352`,`1360`) walks the wrong chain entirely. Currently latent because the block is guarded by `IsInertial()&&IsInertial()` (`1277`) while `MergeLocal` is only invoked for non-inertial sensors — a live trap if that ever changes. *Confidence medium.*
**Trigger** Inertial merges routed through `MergeLocal`, or the guard changing.
**Fix** Use the moving iterator `pKFi = pKFi->mPrevKF/mNextKF`, null-check before `GetMapPoints()`, and walk `mpMergeMatchedKF`'s chain on the merge side. [R1]

#### L5. Covariance observability gate counts edges, not measurement constraints, denying covariance to well-observed stereo frames
**File** `src/Optimizer.cc:1154`. The gate `(nInitialCorrespondences-nBad) >= 6` counts inlier *edges*, but a stereo edge is 3 constraints and a mono edge 2; a well-conditioned 5-stereo-inlier frame (15 constraints) is denied a covariance, while for mono the test is redundant with the eigenvalue check. *Confidence medium.*
**Trigger** A pure-stereo frame with 5 inlier stereo edges and full-rank `H`.
**Fix** Drop the edge-count test (rely on `λ_min` conditioning) or count `2·nMono + 3·nStereo >= 6`. [R10]

#### L6. Early return for `<3` correspondences leaves a stale valid covariance on repeated `PoseOptimization` calls of the same Frame
**File** `src/Optimizer.cc:996`. `if(nInitialCorrespondences<3) return 0;` (`996-997`) skips the covariance block without resetting `mbHasPoseCovariance=false`; in the relocalization retry ladder (`Tracking.cc:3711`/`3727`/`3742`) a later `<3` call on the same Frame leaves an earlier `true` covariance attached to the updated pose. Benign on the common fresh-Frame path. *One confirm / one refute.*
**Trigger** Two-phase relocalization where a later call on the same Frame hits the `<3` early return.
**Fix** Set `pFrame->mbHasPoseCovariance=false;` before the `return 0` (and at any other early return).

#### L7. For frames with `<10` reprojection edges the robust kernels are never cleared, so `H` is evaluated at a robustified optimum
**File** `src/Optimizer.cc:1121`. The block's justification (comment `1121-1122`) assumes the `it==2 setRobustKernel(0)` ran, but the LM loop `break`s when `edges().size()<10` (`1102`) before `it==2`, so for 6–9-correspondence frames the reported pose comes from a robustified optimization while `H` assumes the unweighted LS minimum. *Confidence medium.*
**Trigger** A frame with 6–9 total edges that still passes the `>=6`-inlier covariance gate.
**Fix** Exclude the low-edge case from covariance production, or clear kernels unconditionally before the final `optimize`. [R21]

#### L8. `OptimizeEssentialGraph` inertial (`mPrevKF`) edge built with no `isBad`/vertex-existence guard
**File** `src/Optimizer.cc:5505`. Both essential-graph optimizers add an inertial edge to `pKF->mPrevKF` without checking it is non-bad or has a vertex (vertices exist only for non-bad KFs, `5382`/`1593`), unlike the covisibility (`5569`/`1739`) and spanning-tree edges; a bad `mPrevKF` yields `setVertex(1,nullptr)`. Same omission at `1766`. *Confidence medium; one confirm / one refute.*
**Trigger** Loop/merge pose-graph optimization where some KF's `mPrevKF` is bad.
**Fix** Guard: add the edge only when `prevKF && !prevKF->isBad() && optimizer.vertex(prevKF->mnId)!=nullptr`. [R17][R1]

#### L9. Essential-graph point correction uses a possibly-bad reference keyframe whose Sim3 was never populated
**File** `src/Optimizer.cc:5632`. `nIDr = pMP->GetReferenceKeyFrame()->mnId; Srw=vScw[nIDr]; correctedSwr=vCorrectedSwc[nIDr];` (`5632-5636`) with no `isBad`/null check; bad KFs are skipped at vertex creation (`5382`), so those slots stay default-identity and the point is left uncorrected (or crashes on a null ref KF). *Confidence low; one confirm / one refute.*
**Trigger** A map point whose reference KF is bad at correction time.
**Fix** Skip or fall back to a valid observing KF when `pRefKF` is null/bad; never index `vScw`/`vCorrectedSwc` for a vertex-less KF. [R2]

#### L10. Scale-refinement `InertialOptimization` sets Huber `δ=1.0` on a 9-DOF inertial edge, far tighter than the `sqrt(16.92)` used everywhere else
**File** `src/Optimizer.cc:3537`. `rk->setDelta(1.f)` on each 9-DOF `EdgeInertialGS`, whereas the same residual uses `sqrt(16.92)` (the 0.95 chi2 quantile at 9 DOF) in `LocalInertialBA`/`FullInertialBA`/`MergeInertialBA` (`2699`,`542`,`4286`); `δ=1.0` down-weights nearly every valid IMU link, weakening the scale/gravity estimate this routine exists to compute. *Confidence low; one confirm / one refute.*
**Trigger** `ScaleRefinement` (`LocalMapping.cc:1462`) calling this routine.
**Fix** Use `δ=sqrt(16.92)` as elsewhere, or empirically justify the aggressive delta. [R3]

#### L11. Relocalization `SearchByProjection` never searches the fisheye right camera, so right-only map points cannot be re-found
**File** `src/ORBmatcher.cc:1916`. This overload projects only through the current frame's left camera (`1916`) and queries only the left grid (`1939`), lacking the `Nleft != -1`/`GetRelativePoseTrl()`/`bRight` second pass the frame-to-frame overload has (`1794-1859`); right-only map points are never re-associated during stereo-fisheye relocalization. Combined with H5 this overload is effectively monocular-only. *Confidence medium.*
**Trigger** Relocalization of a stereo-fisheye frame better seen by the right camera.
**Fix** Add a right-camera pass (project via `GetRelativePoseTrl()`, query `GetFeaturesInArea(...,true)`, write into `mvpMapPoints[idx+Nleft]`). [R1]

#### L12. `Sim3Solver::ComputeSim3` runs a debug OpenCV cross-check on every RANSAC iteration
**File** `src/Sim3Solver.cc:384`. `cvnom = Converter::toCvMat(Pr1).dot(toCvMat(P3))` (`384-387`) heap-allocates two `cv::Mat` per RANSAC sample purely to compare against the Eigen `nom` and print a warning; only `nom` is used. Called per iteration at `188`/`261`.
**Trigger** Every loop/merge Sim3 solve.
**Fix** Delete the `cvnom` computation and comparison; keep `nom = (Pr1.array()*P3.array()).sum()`. [R16]

#### L13. `MLPnPsolver::rot2rodrigues` uses `acos(trace/2)` with no domain clamp → NaN silently zeroed
**File** `src/MLPnPsolver.cpp:687`. `wnorm = acos((R.trace()-1)/2)` (`686-688`) can round outside `[-1,1]` and return NaN; the guard `if(wnorm > eps)` is false for NaN, so `omega` stays `(0,0,0)` and a real rotation degrades to an identity Gauss-Newton seed. Degrades, does not crash. *Confidence low.*
**Trigger** `Rout` near identity where `(trace-1)/2` rounds `>1`.
**Fix** Clamp before `acos`: `c = clamp((trace-1)/2, -1, 1)`.

#### L14. ROS-consumed `mTrackedInliers` is stale on failed/lost frames (only refreshed inside `TrackLocalMap`)
**File** `src/System.cc:330`. `System::Track*` copies `GetMatchesInliers()` (`330`,`406`,`486`) every frame, but `mnMatchesInliers` is reset/recomputed only in `TrackLocalMap` (`Tracking.cc:3001`), which runs only when tracking succeeded; on a lost frame it keeps the previous good value, so a health monitor sees a healthy count on a lost frame unless it also checks the state.
**Trigger** A dropped/blurred frame yielding `bOK=false`.
**Fix** Reset `mnMatchesInliers=0` on the failure paths, or set `mTrackedInliers=0` when the published state is LOST/RECENTLY_LOST. [R1]

#### L15. Cross-callback diagnostics state (`track_rate_hz_`, `last_cov_scale_`, `last_track_time_`) is non-atomic while sibling counters were made atomic
**File** `orb_slam3_ros/include/orb_slam3_ros/orb_slam3_node.hpp:154`. These plain doubles/`rclcpp::Time` are written on the image-callback thread and read in the diagnostic-timer callback; `last_tracking_state_`/`frames_tracked_` were made `std::atomic` for exactly this, so the gap is an inconsistent, undocumented reliance on the single-threaded-executor assumption — a genuine race only under a reentrant callback group. *Confidence low; one confirm / one refute.*
**Trigger** A reentrant callback group / MT executor.
**Fix** Make `track_rate_hz_`/`last_cov_scale_` (and `last_track_time_`) atomic or mutex-guard all four.

#### L16. `pose_covariance_diagonal` accepted without a positivity check — can publish a singular/invalid covariance
**File** `orb_slam3_ros/src/orb_slam3_node.cpp:152`. The 6-element diagonal is copied with only a size check (`151-157`) and used at `482`; a zero/negative entry yields a non-SPD 6×6 on `~/odom`, which robot_localization inverts/Cholesky-factors and rejects or NaN-propagates. This is the default (`quality`) producer.
**Trigger** A YAML `pose_covariance_diagonal` with a zero or negative entry.
**Fix** Reject or clamp non-positive entries at load (warn + fall back to defaults). [R19][R20]

#### L17. `package.xml` unconditionally `<depend>`s on `pangolin`, which has no rosdep key
**File** `package.xml:52`. The dependency is declared unconditionally while CMake advertises `USE_PANGOLIN=OFF` as a Pangolin-free build (`CMakeLists.txt:33-35`); colcon resolves package.xml independently of the CMake option, so the headless build still drags in `pangolin`, and `rosdep install` on a fresh clone fails with an unresolved key. *Confidence medium.*
**Trigger** `rosdep install --from-paths` on a checkout lacking `src/Pangolin`, or a headless colcon build.
**Fix** Make the dependency conditional / move it to a viewer variant, or document that `USE_PANGOLIN=OFF` users must remove this depend and vendor `src/Pangolin` first. [R22]

#### L18. Vendored g2o generates `config.h` into the SOURCE tree and installs it from there
**File** `Thirdparty/g2o/CMakeLists.txt:83`. `configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)` (`83`) writes into the checked-in source, and the top-level `install(FILES Thirdparty/g2o/config.h …)` (`CMakeLists.txt:174`) reads it back — fragile on read-only source exports, dirties git, and races parallel/multi-config builds. *Confidence medium.*
**Trigger** A read-only source export (tarball, container COPY, CI), or two concurrent Debug+Release builds.
**Fix** `configure_file(config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)`, add the binary dir to g2o's includes, and install from the binary dir. [R23]

## The new covariance feature — assessment

The covariance pipeline has two halves, and they fail in different ways.

**The ROS-side frame transform is correct.** The producer stores `Σ_cw` as a left-perturbation covariance of `Tcw`, using the g2o Jacobian column order `[ω(rot); υ(trans)]` that matches `VertexSE3Expmap::oplusImpl` and `SE3Quat::exp`. Because `exp(δ)·Tcw ⇒ Twc·exp(−δ)`, `Σ_cw` is simultaneously a right/body perturbation of `Twc` with the same covariance, so the world/left covariance is `Ad_Twc·Σ_cw·Ad_Twcᵀ` with `Ad = [[R,0],[[t]×R,R]]`, `R=Rwc`, `t=twc` (`orb_slam3_node.cpp:426-462`). The adjoint direction, the g2o `[rot,trans] → ROS [trans,rot]` block swap (`453-456`), and SPD preservation were all independently verified as correct. Likewise, the inlier-consistency argument holds: because inliers satisfy `chi2 ≤ 5.991 = δ²`, they sit in the Huber quadratic (unit-weight) region, so the un-robustified `H` is consistent with the final estimate — *except* on the low-edge path (L7), where kernels are never cleared.

**The estimator-side marginal is mathematically the wrong quantity for a metric covariance.** `H = Σ Jᵀ Ω J` is assembled with the 3D landmarks held fixed (motion-only BA), so `H^-1` is the pose covariance *conditioned* on a perfect map, not the marginal pose covariance of the joint pose+structure problem (M5). The marginal requires the Schur term `H_pp − H_pl H_ll^-1 H_lp`, which subtracts a PSD matrix and always yields a *larger* covariance; ORB map points (low-parallax and distant especially) carry the very uncertainty this omits. The in-code comment calling `H^-1` "exact for motion-only BA … no Schur marginalization needed" conflates the conditional with the marginal and should be corrected. The practical consequence is systematic over-confidence published on `~/odom`, where a robot_localization EKF will over-trust weak visual estimates.

Three further defects make the feature unreliable as shipped:
- **Absent in visual-inertial mode (M6):** after IMU init, tracking runs `PoseInertialOptimizationLast{Frame,KeyFrame}`, which never populate `mPoseCovariance`, so `mbTrackedPoseCovarianceValid` is permanently false for every EuRoC/TUM-VI frame — even though those optimizers already assemble an invertible `H.block<6,6>(0,0)`.
- **Non-metric in monocular (M19):** `Σ_cw` is in the map's unobservable gauge scale, so the published `m²` values are really `(map-unit)²` and the lever-arm term scales with `|twc|²` in map units.
- **Discontinuous at the fallback boundary (M20):** frames without a valid marginal drop to an unrelated quality diagonal, jumping 1–3 orders of magnitude and from dense-correlated to axis-aligned — read as sudden de/re-trust by a fusion filter.

Two secondary observability issues round this out: the `>=6`-edge gate mis-counts stereo constraints (L5), and the `<3` early return can leave a stale `mbHasPoseCovariance=true` on a re-optimized Frame (L6). **Bottom line:** the transform math is sound; the covariance is *implemented correctly as a conditional, optimistic, map-scale quantity* but is mislabeled as exact/metric and is unavailable exactly where users most want it (VI mode). Recommended near-term posture: enable `kG2o` only for stereo/RGBD, default `covariance_g2o_scale > 1`, extend the producer into the inertial optimizers, and fix the fallback discontinuity.

## References

[R1] C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós. "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multimap SLAM." *IEEE Transactions on Robotics*, 37(6):1874–1890, 2021. (Secs. III multi-camera/Atlas, IV–C visual-inertial tracking, V inertial init/VIBA, VI–C local inertial BA, VII merging/pose-graph.)

[R2] R. Mur-Artal, J. D. Tardós. "ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras." *IEEE Transactions on Robotics*, 33(5):1255–1262, 2017. (Sec. III tracking/KF culling, Sec. V loop closing & full BA.)

[R3] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza. "On-Manifold Preintegration for Real-Time Visual–Inertial Odometry." *IEEE Transactions on Robotics*, 33(1):1–21, 2017. (Preintegration objects per inter-frame interval; whitened 9-DOF residual information.)

[R4] UZ-SLAMLab/ORB_SLAM3 Issue #730, "Empty IMU measurements vector!!! then segfault" (camera/IMU timeshift). https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/730

[R5] This fork, `docs/ORB_SLAM3_AUDIT.md` (applied-fix audit log; items #9, #11, #12, #16, #30, #32).

[R6] This fork, `docs/THREAD_SAFETY.md` (memory model: flag-bad-never-free; Atlas/Map synchronization; Shutdown wait).

[R7] ISO/IEC 14882, C++ standard: [intro.races] (conflicting non-atomic accesses without happens-before are UB) and [associative.reqmts] (erasing an element invalidates iterators to it).

[R8] C. Campos, J. M. M. Montiel, J. D. Tardós. "Inertial-Only Optimization for Visual-Inertial Initialization." *IEEE ICRA*, 2020.

[R9] J. Solà. "Quaternion kinematics for the error-state Kalman filter." arXiv:1711.02508, 2017. (Sec. 1.3, axis-angle/quaternion singularities at parallel/anti-parallel vectors.)

[R10] R. Hartley, A. Zisserman. *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004. (Sec. 5.2.5 / A6.2 covariance propagation, conditional vs marginal; Sec. 7 minimal PnP observability.)

[R11] M. Kaess, F. Dellaert. "Covariance recovery from a square root information matrix for data association." *Robotics and Autonomous Systems*, 57(12):1198–1210, 2009. (Schur-complement marginal covariance for BA.)

[R12] B. Triggs, P. McLauchlan, R. Hartley, A. Fitzgibbon. "Bundle Adjustment — A Modern Synthesis." *ICCV Workshop on Vision Algorithms*, 1999. (Marginal vs conditional; Schur complement.)

[R13] T. D. Barfoot. *State Estimation for Robotics.* Cambridge University Press, 2017. (Ch. 9, BA covariance recovery.)

[R14] OpenCV `cv::KeyPoint` constructor reference: `KeyPoint(Point2f, float _size, float _angle=-1, float _response=0, int _octave=0, int _class_id=-1)`. https://docs.opencv.org/master/d2/d29/classcv_1_1KeyPoint.html

[R15] J. Kannala, S. S. Brandt. "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses." *IEEE T-PAMI*, 28(8):1335–1340, 2006. (Eq. (6); per-lens radial polynomial `r(θ)`; model valid for `θ>90°`.)

[R16] B. K. P. Horn. "Closed-form solution of absolute orientation using unit quaternions." *JOSA A*, 4(4):629–642, 1987. (Scale `nom/den` in absolute orientation.)

[R17] R. Kümmerle, G. Grisetti, H. Strasdat, K. Konolige, W. Burgard. "g2o: A General Framework for Graph Optimization." *IEEE ICRA*, 2011. (`SparseOptimizer::addEdge` requires all vertices set; `terminate()` polls the force-stop flag before each iteration.)

[R18] H. Strasdat, J. M. M. Montiel, A. J. Davison. "Scale Drift-Aware Large Scale Monocular SLAM." *Robotics: Science and Systems (RSS)*, 2010. (7-DoF Sim3 pose-graph / scale-drift correction.)

[R19] ROS REP-105, "Coordinate Frames for Mobile Platforms." https://www.ros.org/reps/rep-0105.html (`map → odom → base_link` single-parent chain; SLAM publishes `map→odom`; `nav_msgs/Odometry` covariance semantics.)

[R20] T. Moore, D. Stouch. "A Generalized Extended Kalman Filter Implementation for the Robot Operating System (robot_localization)." *IAS-13*, 2014. (Fusion sensitivity to reported covariance consistency; requires SPD pose covariance.)

[R21] Z. Zhang. "Parameter estimation techniques: a tutorial with application to conic fitting." *Image and Vision Computing*, 15(1):59–76, 1997. (M-estimator / robustified vs unweighted covariance.)

[R22] ROS REP-149 "Package Format 3" (condition attributes) and rosdep key-resolution semantics (workspace package names skipped only when present in the workspace). https://www.ros.org/reps/rep-0149.html

[R23] CMake documentation: generated files belong in `CMAKE_CURRENT_BINARY_DIR` (out-of-source build invariant); `configure_file`. https://cmake.org/cmake/help/latest/command/configure_file.html

[R24] UZ-SLAMLab/ORB_SLAM3 `master` source (upstream baseline for findings confirmed identical upstream: `src/Optimizer.cc::LocalInertialBA`, `src/ORBmatcher.cc`, `src/Settings.cc::readCamera2`, `src/KeyFrameDatabase.cc::DetectNBestCandidates`, `src/Map.cc::PreSave`). https://github.com/UZ-SLAMLab/ORB_SLAM3 (verified via raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/).
# ORB-SLAM3 — thread & memory safety notes

Findings and fixes from a concurrency/memory pass over this fork. Fixes that
were **applied** are marked ✅; items left as documented recommendations are ⚠️
(they need dataset validation, deeper lock-ordering work, or an API change that
would break g2o's interface).

## Thread model

| Thread | Created in | Role |
| --- | --- | --- |
| Tracking | caller of `System::Track*` (your ROS callback thread) | frames → pose, new keyframes/map points |
| Local Mapping | `System` ctor (`LocalMapping::Run`) | local BA, map-point & keyframe culling, IMU init |
| Loop Closing | `System` ctor (`LoopClosing::Run`) | place recognition, Sim3, map merging |
| Global BA | spawned by Loop Closing | full BA, then merged back into the live map |
| Viewer | `System` ctor if `use_viewer` | Pangolin GUI (disabled in the ROS/headless build) |

## Lock graph

Per-object mutexes (the guarded data):

- **KeyFrame**: `mMutexPose` (pose/velocity/bias), `mMutexConnections` (covisibility/spanning tree), `mMutexFeatures` (map-point matches), `mMutexMap`.
- **MapPoint**: `mMutexPos`, `mMutexFeatures`, `mMutexMap`, and a **static** `mGlobalMutex`.
- **Map**: `mMutexMap` (containers), `mMutexMapUpdate` (BA vs. tracking exclusion), `mMutexPointCreation` (serializes map-point creation across threads).
- **Atlas**: `mMutexAtlas`. **LocalMapping / LoopClosing**: per-concern flag mutexes.

Observed lock-ordering convention within a KeyFrame method (e.g. `ComputeSceneMedianDepth`, `SetBadFlag`): **`mMutexFeatures` → `mMutexPose`**, and `MapPoint::mGlobalMutex` is the outermost. New locks added by this pass are **leaf** locks (never held while acquiring another) so they cannot invert this order.

## Memory model

ORB-SLAM3 deliberately **does not free** `KeyFrame`/`MapPoint` objects while a map
is live: removal is done by setting a `mbBad` flag and re-checked with `isBad()`,
so stale pointers held by other threads stay valid. `Map::clear()` likewise only
clears the containers (the `delete` lines are commented out). This design avoids
use-after-free at the cost of not reclaiming culled objects until process exit.

## Fixes applied

- ✅ **`System::Shutdown()` now waits for the threads.** The wait loop
  (`while(!LocalMapper.isFinished() || !LoopCloser.isFinished() || isRunningGBA()) usleep`)
  was commented out upstream, so `Shutdown()` returned while Local Mapping / Loop
  Closing kept running — a **crash-on-shutdown** hazard when the owner (e.g. a ROS
  node destructor) then tears down the `System`/Atlas. Restored. *(`System.cc`)*
- ✅ **`LocalMapping::mpCurrentKeyFrame`** is now guarded by a dedicated leaf mutex
  (`mMutexCurrentKF`). `GetCurrKF()` / `GetCurrKFTime()` are called from the
  `System` (user) thread while `LocalMapping::Run` writes the pointer — an unguarded
  cross-thread read (and a potential null-deref). *(`LocalMapping.{h,cc}`)*
- ✅ **`Map::mIsInUse`** is now `std::atomic<bool>` (toggled by Atlas, read by
  `IsInUse()` from other threads; it is not serialized). *(`Map.h`)*
- ✅ (earlier commits) **Leaks removed**: `Relocalization` MLPnP solvers, the
  post-relocalization Frame/Preintegrated triple-leak, and the undefined
  `MLPnPsolver` destructor. **TOCTOU** in `KeyFrame::ComputeSceneMedianDepth`
  (now reads its locked snapshot).

## Verified safe (no change needed)

- **Static ID counters** `MapPoint::nNextId` — serialized: all three `MapPoint`
  constructors hold `mMutexPointCreation` around `mnId = nNextId++`.
  `KeyFrame::nNextId` and `Frame::nNextId` are only incremented on the Tracking
  thread. So the "duplicate id" race does not occur here.

## ⚠️ Remaining recommendations (need validation / API changes)

- **`LocalMapping::mbAbortBA`** is a plain `bool` written by Tracking
  (`InterruptBA`) and read by g2o during `optimize()`. It is passed to
  `g2o::…::setForceStopFlag(bool*)`, whose signature is `bool*` — so it **cannot**
  be made `std::atomic<bool>` without changing g2o. The read/write of an aligned
  `bool` is atomic on x86-64/AArch64, so this is benign in practice; flagged for
  completeness. Same reasoning for the other `bool` stop/finish flags that are
  already mutex-guarded at their call sites.
- **No `System` destructor.** `System` never `delete`s `mpLocalMapper`,
  `mpLoopCloser`, `mpTracker`, `mpAtlas`, or the thread objects, so each
  `System` instance leaks on destruction. One-shot for a normal robot run; it
  accumulates if you repeatedly cleanup→configure a lifecycle node. A proper
  destructor must join the threads (now that `Shutdown()` waits, they will be
  finished) and tear down the Atlas in a safe order — a self-contained follow-up.
- **`KeyFrame::SetBadFlag` / `MapPoint` connection edits** hold the relevant
  mutexes but iterate covisibility structures that peer keyframes may mutate;
  auditing this fully needs a run under **ThreadSanitizer** on a real sequence.
- **`Map` identity `mnId` / `SetId`** — mutated only during map merging; a torn
  read is impossible on 64-bit, so left as-is.

## How to validate

Build with ThreadSanitizer and run a real sequence (EuRoC/TUM-VI or your camera):

```bash
colcon build --packages-select orb_slam3 --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_PANGOLIN=OFF \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread"
```

(Startup/shutdown alone exercises thread creation, the reset paths, and the
`Shutdown()` join; the interesting map-mutation races only appear once tracking
has initialized on feature-rich imagery.)

# LIO SLAM: FAST-LIO2

FAST-LIO2 is a computationally efficient, tightly-coupled LiDAR-Inertial
Odometry system built on an iterated Kalman filter and an incremental
ikd-Tree map, without explicit feature extraction.

![FAST-LIO2 system overview](https://raw.githubusercontent.com/hku-mars/FAST_LIO/main/doc/overview_fastlio2.svg)

- Paper: [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://arxiv.org/abs/2107.06829) (IEEE T-RO / RA-L 2022)
- Upstream: [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) (`ROS2` branch)

In Robotics AI Suite, the upstream tree is a pristine git submodule and Intel
changes ship as patches on top, so FAST-LIO2 can be evaluated as an
alternative LIO backend without forking the reference navigation stack.

> [!IMPORTANT]
> FAST_LIO's [LICENSE](FAST_LIO/LICENSE) file is **GPLv2**. Its
> `package.xml` incorrectly declares `<license>BSD</license>` — that is
> upstream metadata, not the actual terms; treat this package as GPLv2 for
> compliance purposes. For commercial use, contact the upstream authors for
> an alternative license before shipping it in a product.

## Changes to 3rd party source

This work is based on the open-source
[FAST_LIO](https://github.com/hku-mars/FAST_LIO.git) repository (`ROS2`
branch), pinned in [.gitmodules](../../../.gitmodules) at the upstream
commit the patch below applies to.

| Patch | Change |
| ----- | ------ |
| [0001-Add-profiling-instrumentation-new-LiDAR-configs-and-.patch](patches/0001-Add-profiling-instrumentation-new-LiDAR-configs-and-.patch) | New Avia configs; `config/velodyne_generic.yaml` — a Velodyne HDL-32E parameter set for NCLT validation below, with the LiDAR-IMU extrinsic derived from the NCLT dataset paper's own Table 4 sensor calibration (not the UrbanLoco/Point-LIO placeholder it started from — see the comment above `extrinsic_T`/`extrinsic_R` in that file); C++17 + configurable OMP thread count in the build; a preprocess crash fix for Velodyne scans missing a `time` field; and a latency-profiling CSV (below). |

**Profiling**: built behind the `ENABLE_PROFILING` CMake option (off by
default, matching upstream). When enabled, a lock-free ring buffer plus a
dedicated writer thread records per-stage EKF timing (using
`CLOCK_MONOTONIC`, immune to PTP clock steps) to
`FAST_LIO/Log/fast_lio_profiling.csv`.

## Environment setup (Ubuntu 24.04 / ROS 2 Jazzy)

```bash
# 1. Fetch the pristine upstream submodule (--recursive also pulls in
# FAST_LIO's own nested ikd-Tree submodule, required by its CMakeLists.txt)
git submodule update --init --recursive robotics-ai-suite/pipelines/fast-lio2-demo/FAST_LIO

cd robotics-ai-suite/pipelines/fast-lio2-demo/scripts

# 2. One-time host dependencies (needs sudo; safe to re-run)
./install_deps.sh

# 3. Apply the Intel patches from the table above
./apply_patches.sh

# 4. Build fast_lio with colcon
./build.sh
```

All paths, the ROS distro, and the dataset sequence used below are
centralized in [scripts/env.sh](scripts/env.sh) — edit that one file to
retarget a different workspace/sequence; nothing else needs to change.

## Validate without hardware: NCLT dataset replay

No robot or sensor is required to verify the build and measure accuracy:
the `nclt_4` session (2012-01-15) from the public
[NCLT dataset](http://robots.engin.umich.edu/nclt/) (University of
Michigan) is replayed through `fastlio_mapping` and compared against its
SLAM-derived ground truth. NCLT is hosted directly over plain HTTP with no
access request or login — `fetch_nclt.sh` is a straight `wget`.

```bash
./fetch_nclt.sh           # download the Velodyne/IMU/ground-truth files (plain wget, no gating)
./convert_nclt_to_bag.sh  # one-time conversion of the raw files into a standard ROS 2 bag
./run_nclt.sh             # launch fastlio_mapping + `ros2 bag play` the converted bag, records the trajectory
./evaluate_rmse.sh        # evo_ape RMSE vs. ground truth, printed next to the documented baseline

# or, once install_deps.sh has been run once:
./reproduce_all.sh # apply patches -> build -> fetch -> convert -> run -> evaluate, in one command
```

NCLT's raw data isn't a plug-and-play rosbag (custom binary Velodyne format
+ CSV IMU); [scripts/convert_nclt_to_bag.py](scripts/convert_nclt_to_bag.py)
parses it once and writes a standard ROS 2 bag under `BAG_DIR`
([scripts/env.sh](scripts/env.sh)) — `convert_nclt_to_bag.sh` skips this
step on subsequent runs if that bag already exists (pass
`FORCE_CONVERT=true` to redo it). `run_nclt.sh` then replays that bag with
the standard `ros2 bag play`, like every other RAI-suite SLAM demo.

For `nclt_4`, the documented baseline is **8.6 m** RMSE (FAST-LIO2 paper,
arXiv 2107.06829, Table IV — consistent across all tested map sizes,
8.5–8.72 m). The check is one-sided: it passes as long as the freshly
measured RMSE does not exceed that baseline by more than
`RMSE_TOLERANCE_PCT` (20% by default) — a measured RMSE *lower* than the
baseline always passes, since the check exists to catch regressions, not to
flag outperforming the paper's own number.

### Fast iteration on a slice

The full `nclt_4` bag is ~112 minutes and `ros2 bag play` replays it in real
time (no fast-forward), so a full run takes roughly that long. For quicker
iteration, replay only part of the bag via
[scripts/env.sh](scripts/env.sh)'s `PLAY_START_OFFSET_S` /
`PLAY_DURATION_S`:

```bash
PLAY_START_OFFSET_S=0 PLAY_DURATION_S=180 ./run_nclt.sh   # ~3min smoke test
./evaluate_rmse.sh
```

`PLAY_START_OFFSET_S=0` (start from the beginning) is recommended over a
nonzero offset: FAST-LIO2 needs an IMU-init + map-convergence period right
at the start, so skipping ahead produces an unstable trajectory whose RMSE
isn't meaningful. `180s` is enough to get past that init and see a stable
odometry stream while staying fast; bump it toward 600–900s for a more
representative (but still partial) RMSE. `evaluate_rmse.sh` still prints the
measured RMSE for a sliced run but skips the PASS/FAIL check, since the
documented baseline above is for the full sequence only.

### Rviz visualization

`run_nclt.sh` gates `rviz2` behind the `USE_RVIZ` variable in
[scripts/env.sh](scripts/env.sh), off by default so the flow stays headless
over SSH:

```bash
USE_RVIZ=true ./run_nclt.sh   # or: USE_RVIZ=true ./reproduce_all.sh
```

Run this directly on the target machine's own logged-in Ubuntu desktop
session (e.g. on the PTL board's display, not over plain SSH) — rviz2's
point-cloud rendering needs a real GPU display, so X11-forwarding it over
SSH is impractical.

### Reference: running on Intel PTL

`run_nclt.sh` ships a reference core-pinning + frequency-locking setup for
Intel PTL (validated on Core Ultra X7 358H: 4 P-cores `cpu0-3` up to 4700
MHz, 8 E-cores `cpu4-11` up to 3500 MHz, 4 LP-E-cores `cpu12-15` up to 3300
MHz). Core numbering is specific to this SKU — re-check `lscpu -e` before
reusing these defaults on a different PTL SKU or platform.

| Task | Pinned to | Why |
| ---- | --------- | --- |
| `fastlio_mapping` algorithm | LP-E cores `12,13` (`CPUSET_ALGO`) | Keeps the timing-critical LIO thread on isolated cores the general scheduler and rest of the OS don't touch. |
| `ros2 bag play` of the converted NCLT bag | P-core `1` (`CPUSET_BAG`) | Replaying the pre-converted bag is bursty I/O + decode work; a dedicated P-core keeps it from stealing cycles from the algorithm cores. |
| `rviz2` (when `USE_RVIZ=true`) | P-core `2` (`CPUSET_RVIZ`) | Point-cloud rendering is bursty GUI work best kept off the algorithm's isolated cores; a P-core has the headroom for it. |

`run_nclt.sh` wraps the algorithm and `ros2 bag play` with `taskset
-c` and, best-effort, a `sudo -n chrt -f 85` SCHED_FIFO priority-85 run,
whenever the matching `CPUSET_*` variable in
[scripts/env.sh](scripts/env.sh) is non-empty (the default). `rviz2` gets
`taskset` pinning only, no realtime priority. If `sudo -n` isn't usable (no
passwordless sudoers entry for `chrt`), the script warns and continues
unprioritized rather than failing the run. To disable pinning for a given
task, blank out its variable in `env.sh` (e.g. `CPUSET_ALGO=""`).

Because raising a process to SCHED_FIFO requires elevating before its
final exec, the algorithm and `ros2 bag play` processes run as **root**
whenever RT-prioritized this way — files they write may end up root-owned.
`rviz2` is not launched via `sudo`/`chrt` and stays as the invoking user.

For apples-to-apples benchmarking, lock every core's governor and min/max
frequency (and, as a stronger hardware-level backstop, the HWP MSR
request) before measuring:

```bash
sudo ./limit_ptl_cores.sh
```

This requires root and prints a per-core summary of the governor/min/max
frequency actually applied. Its targets (`FREQ_P_CORES`/`FREQ_E_CORES`/
`FREQ_LPE_CORES`, `FREQ_*_MAX`/`FREQ_*_MIN`, `CPU_MODE_P`/`CPU_MODE_E`) are
also in `env.sh`.

## Limitations / non-goals

- Validated here: functional LIO operation and pose-tracking accuracy
  (RMSE) against the public NCLT baseline, on a Velodyne-class LiDAR.
- `fast_lio`'s build unconditionally depends on `livox_ros_driver2` (and
  transitively Livox-SDK2), even though this pipeline only ever runs the
  Velodyne/NCLT path — confirmed in `CMakeLists.txt`/`package.xml`, not a
  choice made by this integration.
- The NCLT Velodyne/IMU binary-format parsing in
  [scripts/convert_nclt_to_bag.py](scripts/convert_nclt_to_bag.py) and the
  ground-truth parsing in
  [scripts/extract_nclt_gt.py](scripts/extract_nclt_gt.py) were verified
  against NCLT's own devkit scripts and a real (partial) download of the
  `nclt_4` session's raw files — but a full end-to-end `reproduce_all.sh`
  run (through `fastlio_mapping` and RMSE evaluation) has not been
  exercised in the environment this was authored in, which lacks a ROS 2
  install; validate on your own machine or PTL before relying
  on the PASS/FAIL result.
- Only `nclt_4` has a confirmed session date and documented baseline;
  `nclt_5`–`nclt_10` are structural placeholders in `scripts/env.sh` for
  future extension, not yet populated.
- NCLT's terms of use should be checked on the
  [dataset's own page](http://robots.engin.umich.edu/nclt/) before
  redistributing any downloaded data.
- GPLv2 licensing (see callout above) applies to the upstream code as-is;
  this integration does not change that.

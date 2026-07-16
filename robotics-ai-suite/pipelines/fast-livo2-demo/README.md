# LIVO SLAM: FAST-LIVO2

FAST-LIVO2 is a direct (feature-less) LiDAR-Inertial-Visual Odometry system:
it fuses a LiDAR-Inertial pose estimate with dense visual-inertial tracking on
raw image patches, avoiding explicit feature extraction/matching. It targets
real-time onboard localization and mapping, including in visually- or
geometrically-degraded environments where either sensor alone struggles.

<div align="center">
    <img src="FAST-LIVO2/pics/Framework.png" width="80%">
</div>

- Paper: [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035) (accepted, T-RO'24)
- Paper: [FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)
- Upstream: [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)

In Robotics AI Suite, the upstream tree is a pristine git submodule and Intel
changes ship as patches on top, so FAST-LIVO2 can be evaluated as an
alternative SLAM backend without forking the reference navigation stack.

> [!IMPORTANT]
> FAST-LIVO2 is released under **GPLv2**. For commercial use, contact the
> upstream authors (see [FAST-LIVO2/README.md](FAST-LIVO2/README.md#5-license))
> for an alternative license before shipping it in a product.

## Changes to 3rd party source

This work is based on the open-source
[FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2.git) repository, pinned in
[.gitmodules](../../../.gitmodules) at the upstream commit the patches below
apply to.

| Patch | Enhancement |
| ----- | ----------- |
| [0001-Stop-VIO-waiting-on-the-LiDAR-buffer-in-LIVO-mode.patch](patches/0001-Stop-VIO-waiting-on-the-LiDAR-buffer-in-LIVO-mode.patch) | Removes a LiDAR-buffer precondition that gated every VIO update even though the VIO step never reads the LiDAR queue. Measured: ~30 ms of gate wait removed per VIO update at 10 Hz LiDAR input ([src/LIVMapper.cpp](FAST-LIVO2/src/LIVMapper.cpp)). |
| [0002-Port-to-ROS2-and-bring-up-Mid-360-D415-on-A2W.patch](patches/0002-Port-to-ROS2-and-bring-up-Mid-360-D415-on-A2W.patch) | Ports the codebase and launch files from ROS1/catkin to ROS2/ament (validated on Humble and Jazzy, see [FAST-LIVO2/README_ROS2.md](FAST-LIVO2/README_ROS2.md)); adds a Livox Mid-360 + RealSense D415 sensor profile; adds an optional per-frame LIO/VIO timing CSV export gated behind `-DENABLE_PERFRAME_TIMING=ON` (off by default) for latency analysis; fixes a DDS parameter-discovery race and an inverted Mid-360 mount orientation. |
| [0003-Size-OMP-thread-count-from-runtime-CPU-affinity.patch](patches/0003-Size-OMP-thread-count-from-runtime-CPU-affinity.patch) | Sizes the LIO/VIO OMP thread count from the process's actual CPU affinity at startup (`sched_getaffinity`) instead of the build-time total host core count, so pinning `fast_livo2` to a smaller cpu set via `taskset -c` (`CPUSET_ALGO` in `scripts/env.sh`) no longer oversubscribes the pinned cores with more OMP threads than they can run. Falls back to the original `ProcessorCount`-based build-time default when the process isn't affinity-restricted. |

## Environment setup (Ubuntu 24.04 / ROS 2 Jazzy, Intel Core Ultra / PTL)

Full one-time host prerequisites (system packages, Livox-SDK2, Sophus,
vikit_common) are documented once in
[FAST-LIVO2/README_ROS2.md](FAST-LIVO2/README_ROS2.md) — the steps below just
automate exactly those commands via [scripts](scripts):

```bash
# 1. Fetch the pristine upstream submodule
git submodule update --init robotics-ai-suite/pipelines/fast-livo2-demo/FAST-LIVO2

cd robotics-ai-suite/pipelines/fast-livo2-demo/scripts

# 2. One-time host dependencies (needs sudo; safe to re-run)
./install_deps.sh

# 3. Apply the Intel patches from the table above
./apply_patches.sh

# 4. Build fast_livo2 with colcon
./build.sh
```

All paths, the ROS distro, and the dataset sequence used below are
centralized in [scripts/env.sh](scripts/env.sh) — edit that one file to
retarget a different workspace/sequence; nothing else needs to change.

## Validate without hardware: NTU VIRAL dataset replay

No robot or sensor is required to verify the build and measure accuracy: the
Ouster OS1 + camera + IMU `eee_03` sequence from the public
[NTU VIRAL dataset](https://ntu-aris.github.io/ntu_viral_dataset/)
(Nguyen et al., *NTU VIRAL: A Visual-Inertial-Ranging-Lidar Dataset, From an
Aerial Vehicle Viewpoint*, IJRR 2022) is replayed through the same
`fast_livo2` binary and compared against surveyed ground truth.

```bash
./fetch_ntu_viral.sh          # download eee_03 bag + convert to ROS2 (auto; manual fallback for unlisted sequences)
./run_ntu_viral.sh            # launch fast_livo2 + play back the bag, records the trajectory
./evaluate_rmse.sh            # evo_ape RMSE vs. ground truth, printed next to the documented baseline

# or, once install_deps.sh has been run once:
./reproduce_all.sh            # apply patches -> build -> fetch -> run -> evaluate, in one command
```

`evaluate_rmse.sh` reproduces the same PRISM-frame conversion and `evo_ape`
comparison already checked into
[FAST-LIVO2/Log/result/ntu_viral/](FAST-LIVO2/Log/result/ntu_viral/), whose
`README.md` documents reference RMSE for all nine sequences from prior runs.
For `eee_03`, the documented baseline is **2.61 cm**; the script passes when
the freshly measured RMSE is within `RMSE_TOLERANCE_PCT` (20% by default,
see [scripts/env.sh](scripts/env.sh)) of that baseline — not a specific
improvement claim.

### Rviz visualization

`run_ntu_viral.sh` gates `rviz2` behind the `USE_RVIZ` variable in
[scripts/env.sh](scripts/env.sh), off by default so the flow stays headless
over SSH:

```bash
USE_RVIZ=true ./run_ntu_viral.sh   # or: USE_RVIZ=true ./reproduce_all.sh
```

Run this directly on the target machine's own logged-in Ubuntu desktop
session (e.g. on the PTL board's display, not over plain SSH) — rviz2's
point-cloud rendering needs a real GPU display, so X11-forwarding it over
SSH is impractical. It opens with the
[ntu_viral.rviz](FAST-LIVO2/rviz_cfg/ntu_viral.rviz) config, showing the
live point cloud and pose trajectory as the bag plays back.

### Reference: running on Intel PTL

`run_ntu_viral.sh` ships a reference core-pinning + frequency-locking
setup for Intel PTL (validated on Core Ultra X7 358H: 4 P-cores `cpu0-3` up
to 4700 MHz, 8 E-cores `cpu4-11` up to 3500 MHz, 4 LP-E-cores `cpu12-15` up
to 3300 MHz). Core numbering is specific to this SKU — re-check `lscpu -e`
before reusing these defaults on a different PTL SKU or platform.

| Task | Pinned to | Why |
| ---- | --------- | --- |
| `fast_livo2` algorithm | LP-E cores `12,13` (`CPUSET_ALGO`) | Keeps the timing-critical LIO/VIO threads on isolated cores the general scheduler and rest of the OS don't touch. |
| `ros2 bag play` | P-core `1` (`CPUSET_BAG`) | Bag replay is bursty I/O + deserialization work; a dedicated P-core keeps it from stealing cycles from the algorithm cores. |
| `rviz2` (when `USE_RVIZ=true`) | P-core `2` (`CPUSET_RVIZ`) | Point-cloud rendering is bursty GUI work best kept off the algorithm's isolated cores; a P-core has the headroom for it. |

These three assignments are independent of each other and of `USE_RVIZ`:
the algorithm always runs on `12,13` whether or not rviz is enabled, and
`rviz2` always runs as its own separate process on P-core `2` (never as a
child of the algorithm's `ros2 launch`, so it never shares or inherits the
algorithm's affinity).

This assumes `cpu12,13` (the isolated core set used for real-time work)
have already been isolated from the general kernel scheduler at the
platform/BKC level (e.g. `isolcpus=`/equivalent boot config) — that
isolation is out of scope for this repo and expected to already be in
place on the target machine.

`run_ntu_viral.sh` automatically wraps the `fast_livo2` launch and
`ros2 bag play` with `taskset -c` and, best-effort, a
`sudo -n chrt -f 85` SCHED_FIFO priority-85 run, whenever the matching
`CPUSET_*` variable in [scripts/env.sh](scripts/env.sh) is non-empty (the
default). `rviz2` gets `taskset` pinning only, no realtime priority — GUI
rendering work shouldn't run SCHED_FIFO. If `sudo -n` isn't usable (no
passwordless sudoers entry for `chrt`), the script warns and continues
unprioritized rather than failing the run. To disable pinning for a given
task, blank out its variable in `env.sh` (e.g. `CPUSET_ALGO=""`).

`fast_livo2` sizes its LIO/VIO OMP thread count from its actual runtime CPU
affinity (detected via `sched_getaffinity` at startup, see patch 0003 above),
not a value baked in at build time — so changing `CPUSET_ALGO` takes effect
on the next run, no rebuild needed. Running unpinned (`CPUSET_ALGO=""`) falls
back to the original build-time `ProcessorCount`-based default.

Because raising a process to SCHED_FIFO requires elevating before its
final exec (with no way to drop back to the invoking user afterward within
one command line), the `fast_livo2` algorithm and `ros2 bag play` run as
**root** whenever RT-prioritized this way — files they write (e.g.
`Log/result/*.txt`) may end up root-owned. `rviz2` is not launched via
`sudo`/`chrt` and stays as the invoking user.

For apples-to-apples benchmarking, lock every core's governor and
min/max frequency (and, as a stronger hardware-level backstop, the HWP
MSR request) before measuring:

```bash
sudo ./limit_ptl_cores.sh
```

This requires root (it writes to `/sys/devices/system/cpu/*/cpufreq` and,
if `msr-tools` is installed, MSR `0x774`) and prints a per-core summary of
the governor/min/max/current frequency actually applied. Its targets
(`FREQ_P_CORES`/`FREQ_E_CORES`/`FREQ_LPE_CORES`, `FREQ_*_MAX`/`FREQ_*_MIN`,
`CPU_MODE_P`/`CPU_MODE_E`) are also in `env.sh`.

## Limitations / non-goals

- Validated here: functional SLAM operation and pose-tracking accuracy
  (RMSE) against the public NTU VIRAL baseline.
- Sensor assumption: a synchronized LiDAR + camera + IMU stream (native
  Livox format for the Mid-360 profile, or a standard rosbag as in the NTU
  VIRAL flow above).
- Real-robot bring-up (Mid-360 + D415 on A2W) uses the config shipped in
  patch 0002 (`config/mid360-a2w*.yaml`) but requires that physical hardware
  and is not exercised by this reproduce flow.
- GPLv2 licensing (see callout above) applies to the upstream code as-is;
  this integration does not change that.

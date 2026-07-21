#!/usr/bin/env bash
# Launch fastlio_mapping on the configured NCLT sequence and replay it via
# `ros2 bag play` against the bag scripts/convert_nclt_to_bag.sh produced
# (NCLT has no plug-and-play rosbag of its own - that one-time conversion
# step parses NCLT's raw binary/CSV files instead).
# Produces an estimated trajectory at ${RESULTS_DIR}/<sequence>_est_tum.txt
# (via record_odometry_tum.py, consumed by evaluate_rmse.sh).
#
# Usage: ./run_nclt.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

SEQ="${NCLT_SEQUENCE}"

# PIDs of processes ptl_wrap launched via the sudo+chrt path (see there) -
# these run as root, so cleanup's kill/wait below must go through
# `sudo -n kill` for them; a plain `kill` from this unprivileged script
# fails with EPERM (silently, since callers historically did `|| true`).
declare -A PTL_ROOT_PID=()

if [[ ! -d "${DATASET_DIR}" ]] || [[ -z "$(find "${DATASET_DIR}" -name velodyne_hits.bin 2>/dev/null)" ]]; then
  echo "NCLT dataset not found at ${DATASET_DIR}. Run ./fetch_nclt.sh first." >&2
  exit 1
fi
if [[ ! -d "${BAG_DIR}" ]]; then
  echo "No converted bag at ${BAG_DIR}. Run ./convert_nclt_to_bag.sh first." >&2
  exit 1
fi

# ROS 2's setup.bash references internal ament/colcon trace variables that
# are never exported with a default, so it's incompatible with `set -u`;
# disable nounset just around sourcing it.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

# ptl_wrap <cpuset> <rt:0|1> <cmd...>
# Backgrounds "<cmd...>", pinned to CPU list <cpuset> via taskset (skipped
# if <cpuset> is empty), and - only when <rt> is 1 - best-effort run under
# SCHED_FIFO priority 85 via `sudo -n chrt -f 85 <cmd...>`. This is one
# command line, not a launch-then-reprioritize: chrt's `-p` flag only
# re-prioritizes an already-running PID, so instead `chrt -f 85 taskset -c
# <cpuset> <cmd...>` is chained as a single command - chrt execs taskset
# under SCHED_FIFO-85, taskset sets CPU affinity and execs <cmd...>, which
# keeps the FIFO-85 policy set before that exec. Net effect: an
# RT-prioritized task runs as root end-to-end (sudo -n elevates before the
# final exec, with no way to drop back to the invoking user within one
# command line) - see README.md "Reference: running on Intel PTL".
# Sets PTL_LAST_PID (a `pid=$(ptl_wrap ...)` return would lose the
# background job to a subshell, since command substitution runs in one).
ptl_wrap() {
  local cpuset="$1" rt="$2"; shift 2
  if [[ -z "${cpuset}" ]]; then
    "$@" &
  elif [[ "${rt}" == "1" ]] && sudo -n true 2>/dev/null; then
    # sudo's `secure_path`/`env_reset` strip PATH, LD_LIBRARY_PATH,
    # AMENT_PREFIX_PATH, PYTHONPATH etc. before exec'ing chrt, so a bare
    # `sudo -n chrt ... taskset ... ros2 ...` fails to find/import ROS 2 at
    # all - none of the ROS 2 environment sourced above survives into the
    # sudo'd process. Rather than fight sudo's env allowlist var-by-var,
    # re-source both setup.bash files inside a `bash -c` that
    # sudo/chrt/taskset exec, then `exec` the real command from there so
    # the RT priority + affinity set by chrt/taskset (both preserved across
    # exec) still apply to it. ROS_DOMAIN_ID/RMW_IMPLEMENTATION are
    # exported by env.sh, not restored by the ROS setup.bash files, so
    # re-sourcing those alone doesn't restore them - env_reset wipes them
    # and the sudo'd process silently falls back to domain 0 with the
    # default RMW, putting it on a different DDS domain than any
    # non-sudo'd process (e.g. rviz2). Re-export both explicitly, from this
    # shell's values, before exec'ing.
    local cmd_str ros_env_str=""
    cmd_str="$(printf '%q ' "$@")"
    if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
      ros_env_str+="export ROS_DOMAIN_ID=$(printf '%q' "${ROS_DOMAIN_ID}")
"
    fi
    if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
      ros_env_str+="export RMW_IMPLEMENTATION=$(printf '%q' "${RMW_IMPLEMENTATION}")
"
    fi
    sudo -n chrt -f 85 taskset -c "${cpuset}" bash -c "
      set +u
      ${ros_env_str}
      source '/opt/ros/${ROS_DISTRO}/setup.bash'
      source '${WS_DIR}/install/setup.bash'
      set -u
      exec ${cmd_str}" &
    PTL_LAST_PID=$!
    PTL_ROOT_PID["${PTL_LAST_PID}"]=1
    return 0
  else
    if [[ "${rt}" == "1" ]]; then
      echo "WARN: 'sudo -n' unavailable (no NOPASSWD sudoers entry for chrt) - running pinned to cpu ${cpuset} without realtime priority" >&2
    fi
    taskset -c "${cpuset}" "$@" &
  fi
  PTL_LAST_PID=$!
}

# ptl_pid_tree <pid> - print <pid> and all of its descendants (recursive,
# via `pgrep -P`), one per line. ptl_wrap's root-owned chain (sudo -> chrt
# -> taskset -> bash -c -> exec'd command) only replaces its own process
# image via exec, but some launched tools (e.g. `ros2 run`) spawn the real
# binary as a SEPARATE child via subprocess rather than exec - a signal to
# the top PID alone never reaches that grandchild, which is then orphaned
# (reparented to pid 1) and keeps running, holding this script's
# stdout/stderr pipe open forever (observed in practice: a leaked
# fastlio_mapping process kept `tee reproduce_all.log` from ever seeing
# EOF, hanging the whole pipeline indefinitely after everything else had
# already finished). Walk the whole tree so cleanup actually terminates
# every process a wrapped command started, not just its wrapper.
ptl_pid_tree() {
  local pid="$1" child
  echo "${pid}"
  for child in $(pgrep -P "${pid}" 2>/dev/null); do
    ptl_pid_tree "${child}"
  done
}

# Signal `pid` and all its descendants, going through `sudo -n` when `pid`
# is one of the root-owned PIDs ptl_wrap recorded above - a plain `kill` on
# those (and their equally root-owned descendants) fails with EPERM from
# this unprivileged script.
ptl_kill() {
  local pid="$1" sig="$2" p
  for p in $(ptl_pid_tree "${pid}"); do
    if [[ -n "${PTL_ROOT_PID[${pid}]:-}" ]]; then
      sudo -n kill "-${sig}" "${p}" 2>/dev/null || kill "-${sig}" "${p}" 2>/dev/null || true
    else
      kill "-${sig}" "${p}" 2>/dev/null || true
    fi
  done
}

# Same root-vs-plain distinction as ptl_kill: `kill -0` on a root-owned pid
# fails with EPERM from this unprivileged script even while it's alive, so
# liveness checks below need the same sudo fallback - and the same
# whole-tree walk, since a live grandchild counts as "still alive" too.
ptl_pid_alive() {
  local pid="$1" p
  for p in $(ptl_pid_tree "${pid}"); do
    if [[ -n "${PTL_ROOT_PID[${pid}]:-}" ]]; then
      sudo -n kill -0 "${p}" 2>/dev/null && return 0
    else
      kill -0 "${p}" 2>/dev/null && return 0
    fi
  done
  return 1
}

# ptl_wait_dead <pid> <max tenths-of-a-second> - poll ptl_pid_alive rather
# than blocking on it, so a stuck process bounds this to a fixed timeout
# instead of hanging forever.
ptl_wait_dead() {
  local pid="$1" limit="$2" waited=0
  while ptl_pid_alive "${pid}" && (( waited < limit )); do
    sleep 0.1
    (( ++waited ))
  done
  ! ptl_pid_alive "${pid}"
}

# Send SIGTERM to `pid`, wait up to ~10s for it to exit, escalate to
# SIGKILL and wait up to ~5s more, then reap it.
ptl_stop() {
  local pid="$1"
  [[ -z "${pid}" ]] && return 0
  ptl_kill "${pid}" TERM
  ptl_wait_dead "${pid}" 100 && { wait "${pid}" 2>/dev/null || true; return 0; }
  ptl_kill "${pid}" KILL
  if ptl_wait_dead "${pid}" 50; then
    wait "${pid}" 2>/dev/null || true
  else
    echo "WARN: could not stop pid ${pid} - check the sudoers NOPASSWD policy covers 'kill', not just 'chrt'" >&2
  fi
}

mkdir -p "${RESULTS_DIR}"
RESULT_FILE="${RESULTS_DIR}/${SEQ}_est_tum.txt"
rm -f "${RESULT_FILE}"

CONFIG_PATH="${WS_DIR}/install/fast_lio/share/fast_lio/config"
CONFIG_FILE="velodyne_generic.yaml"

echo "==> Launching fastlio_mapping for sequence ${SEQ}"
# rviz2 is always launched (if at all) as its own process below, so it never
# inherits fastlio_mapping's own taskset affinity. See README.md "Reference:
# running on Intel PTL".
ptl_wrap "${CPUSET_ALGO}" 1 \
  ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "${CONFIG_PATH}/${CONFIG_FILE}" \
  -p "common.lid_topic:=/velodyne_points" \
  -p "common.imu_topic:=/imu/data" \
  -p "pcd_save.pcd_save_en:=false" \
  -p "use_sim_time:=false"
ALGO_PID="${PTL_LAST_PID}"
RVIZ_PID=""
PUB_PID=""
REC_PID=""

# Always stop every process and remove nothing (no scratch files here) on
# exit, even if a step below fails - otherwise (with `set -e`) this script
# would abort before reaching cleanup, leaving processes running as
# orphans. Goes through ptl_stop (sudo-aware kill + bounded wait + SIGKILL
# escalation) since fastlio_mapping and `ros2 bag play` run as root when
# RT-prioritized (see ptl_wrap above), and a plain `kill` from this
# unprivileged script silently fails on them (EPERM).
cleanup() {
  ptl_stop "${ALGO_PID}"
  ptl_stop "${REC_PID}"
  ptl_stop "${RVIZ_PID}"
  ptl_stop "${PUB_PID}"
}
trap cleanup EXIT

echo "==> Recording /Odometry to ${RESULT_FILE}"
python3 "${SCRIPT_DIR}/record_odometry_tum.py" --topic /Odometry --out "${RESULT_FILE}" &
REC_PID=$!

if [[ "${USE_RVIZ}" == "true" ]]; then
  echo "==> Launching rviz2"
  # No SCHED_FIFO here (rt=0): rviz2 is GUI/rendering work off the timing
  # -critical path; priority-85 FIFO on a process that can block on GL/X11
  # calls risks starving other tasks on its core.
  ptl_wrap "${CPUSET_RVIZ}" 0 rviz2 -d "${FASTLIO_SRC}/rviz/fastlio.rviz"
  RVIZ_PID="${PTL_LAST_PID}"
fi

sleep 5
echo "==> Playing back NCLT ${SEQ} bag from ${BAG_DIR}"
# `ros2 bag play` replays at ~1x recorded speed and this script blocks on
# it until playback finishes, so print the bag's own recorded duration up
# front - otherwise a multi-minute NCLT sequence produces no further log
# output until playback ends, which looks stuck rather than just replaying
# in real time.
BAG_DURATION="$(ros2 bag info "${BAG_DIR}" 2>/dev/null | sed -n 's/^ *Duration: *//p')"
PLAY_ARGS=()
if [[ -n "${PLAY_START_OFFSET_S}" ]]; then
  PLAY_ARGS+=(--start-offset "${PLAY_START_OFFSET_S}")
fi
if [[ -n "${PLAY_DURATION_S}" ]]; then
  echo "==> Bag duration: ${BAG_DURATION:-unknown}; playing a ${PLAY_DURATION_S}s slice starting at offset ${PLAY_START_OFFSET_S:-0}s (real time)."
else
  echo "==> Bag duration: ${BAG_DURATION:-unknown} - playback runs in real time, so expect roughly that long before the next log line."
fi
ptl_wrap "${CPUSET_BAG}" 1 \
  ros2 bag play "${BAG_DIR}" "${PLAY_ARGS[@]}"
PUB_PID="${PTL_LAST_PID}"
if [[ -n "${PLAY_DURATION_S}" ]]; then
  # Let it run for the requested slice, then stop it explicitly rather than
  # waiting for EOF - ptl_stop is a harmless no-op if playback already
  # finished on its own (e.g. slice longer than what's left in the bag).
  ptl_wait_dead "${PUB_PID}" $(( PLAY_DURATION_S * 10 )) || true
  ptl_stop "${PUB_PID}"
else
  wait "${PUB_PID}"
fi

echo "==> Playback finished, stopping fastlio_mapping"
sleep 2  # let the last odometry messages land before the recorder is stopped

if [[ -s "${RESULT_FILE}" ]]; then
  echo "==> Trajectory written to ${RESULT_FILE}"
else
  echo "No trajectory written to ${RESULT_FILE} - check the fastlio_mapping log output above." >&2
  exit 1
fi

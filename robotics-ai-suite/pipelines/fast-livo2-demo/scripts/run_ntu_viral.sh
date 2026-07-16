#!/usr/bin/env bash
# Launch fast_livo2 on the configured NTU VIRAL sequence and play back the
# converted ROS 2 bag. Produces a TUM-format trajectory at
# FAST-LIVO2/Log/result/<sequence>.txt (consumed by evaluate_rmse.sh).
#
# Usage: ./run_ntu_viral.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

SEQ="${NTU_VIRAL_SEQUENCE}"
BAG_ROS2="${DATASET_DIR}/${SEQ}"

# PIDs of processes ptl_wrap launched via the sudo+chrt path (see there) -
# these run as root, so cleanup's kill/wait below must go through
# `sudo -n kill` for them; a plain `kill` from this unprivileged script
# fails with EPERM (silently, since callers historically did `|| true`).
declare -A PTL_ROOT_PID=()

if [[ ! -d "${BAG_ROS2}" ]]; then
  echo "Converted bag not found at ${BAG_ROS2}. Run ./fetch_ntu_viral.sh first." >&2
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
# re-prioritizes an already-running PID (`chrt -f -p 85 taskset -c ... cmd`
# fails immediately with "invalid PID argument: 'taskset'"), so instead
# `chrt -f 85 taskset -c <cpuset> <cmd...>` is chained as a single command -
# chrt execs taskset under SCHED_FIFO-85, taskset sets CPU affinity and
# execs <cmd...>, which keeps the FIFO-85 policy set before that exec.
# Net effect: an RT-prioritized task runs as root end-to-end (sudo -n
# elevates before the final exec, with no way to drop back to the invoking
# user within one command line) - see README.md "Reference: running on
# Intel PTL".
# Sets PTL_LAST_PID (a `pid=$(ptl_wrap ...)` return would lose the
# background job to a subshell, since command substitution runs in one).
ptl_wrap() {
  local cpuset="$1" rt="$2"; shift 2
  if [[ -z "${cpuset}" ]]; then
    "$@" &
  elif [[ "${rt}" == "1" ]] && sudo -n true 2>/dev/null; then
    # sudo's `secure_path`/`env_reset` (see /etc/sudoers) strip PATH,
    # LD_LIBRARY_PATH, AMENT_PREFIX_PATH, PYTHONPATH etc. before exec'ing
    # chrt, so a bare `sudo -n chrt ... taskset ... ros2 ...` fails with
    # "taskset: failed to execute ros2: No such file or directory" (or,
    # once ros2 is found some other way, ImportError on librcl*.so) - none
    # of the ROS 2 environment sourced above survives into the sudo'd
    # process. Rather than fight sudo's env allowlist var-by-var, re-source
    # both setup.bash files inside a `bash -c` that sudo/chrt/taskset exec,
    # then `exec` the real command from there so the RT priority + affinity
    # set by chrt/taskset (both preserved across exec) still apply to it.
    # ROS_DOMAIN_ID/RMW_IMPLEMENTATION are set in this machine's ~/.bashrc,
    # not by the ROS setup.bash files, so re-sourcing those alone doesn't
    # restore them - env_reset wipes them and the sudo'd process silently
    # falls back to domain 0 with the default RMW. That put it on a
    # different DDS domain than rviz2 (launched without sudo, rt=0, so it
    # keeps this shell's env) - rviz2 could never discover this process's
    # publishers, even though the process itself ran fine and produced a
    # valid trajectory. Re-export both explicitly, from this shell's
    # values, before exec'ing.
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

# Signal `pid`, going through `sudo -n` when it's one of the root-owned
# PIDs ptl_wrap recorded above - a plain `kill` on those fails with EPERM
# from this unprivileged script, which previously left fast_livo2 running
# forever (looping on rclcpp::ok(), never seeing the signal) after bag
# playback finished.
ptl_kill() {
  local pid="$1" sig="$2"
  if [[ -n "${PTL_ROOT_PID[${pid}]:-}" ]]; then
    sudo -n kill "-${sig}" "${pid}" 2>/dev/null || kill "-${sig}" "${pid}" 2>/dev/null || true
  else
    kill "-${sig}" "${pid}" 2>/dev/null || true
  fi
}

# Same root-vs-plain distinction as ptl_kill: `kill -0` on a root-owned pid
# fails with EPERM from this unprivileged script even while it's alive, so
# liveness checks below need the same sudo fallback.
ptl_pid_alive() {
  local pid="$1"
  if [[ -n "${PTL_ROOT_PID[${pid}]:-}" ]]; then
    sudo -n kill -0 "${pid}" 2>/dev/null
  else
    kill -0 "${pid}" 2>/dev/null
  fi
}

# ptl_wait_dead <pid> <max tenths-of-a-second> - poll ptl_pid_alive rather
# than blocking on it, so a stuck process bounds this to a fixed timeout
# instead of hanging forever.
ptl_wait_dead() {
  local pid="$1" limit="$2" waited=0
  # Pre-increment: post-increment's `(( waited++ ))` evaluates to the
  # pre-increment value, which is 0 on the first pass - under `set -e` a
  # standalone `(( expr ))` that evaluates to 0 counts as command failure
  # and would abort the whole script right here.
  while ptl_pid_alive "${pid}" && (( waited < limit )); do
    sleep 0.1
    (( ++waited ))
  done
  ! ptl_pid_alive "${pid}"
}

# Send SIGTERM to `pid`, wait up to ~10s for it to exit, escalate to
# SIGKILL and wait up to ~5s more, then reap it. Every wait is bounded by
# ptl_wait_dead's polling (never a blocking `wait`) so a process this
# script genuinely cannot signal (e.g. a sudoers policy that permits
# `chrt` but not `kill`) logs a warning instead of hanging the script
# forever the way the old plain-`kill`-then-`wait` trap did.
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

mkdir -p "${FASTLIVO2_SRC}/Log/result"
RESULT_FILE="${FASTLIVO2_SRC}/Log/result/${SEQ}.txt"
rm -f "${RESULT_FILE}"

# NTU_VIRAL.yaml defaults evo/seq_name to "eee_01"; point it at the
# configured sequence via a scratch copy instead of editing the tracked file.
PARAMS_FILE="$(mktemp --suffix=.yaml)"
sed "s/seq_name: \"eee_01\"/seq_name: \"${SEQ}\"/" \
  "${FASTLIVO2_SRC}/config/NTU_VIRAL.yaml" > "${PARAMS_FILE}"

echo "==> Launching fast_livo2 for sequence ${SEQ}"
# rviz2 is always launched (if at all) as its own process below, never via
# use_rviz:=true here - that would spawn it as a child Node inside this same
# ros2 launch process, inheriting fast_livo2's own taskset affinity with no
# way to give it CPUSET_RVIZ independently. See README.md "Reference:
# running on Intel PTL".
ptl_wrap "${CPUSET_ALGO}" 1 \
  ros2 launch fast_livo2 mapping_ouster_ntu.launch.py \
  use_rviz:=false \
  avia_params_file:="${PARAMS_FILE}"
LIVO_PID="${PTL_LAST_PID}"
RVIZ_PID=""
BAG_PID=""
# Always stop fast_livo2 (and rviz2/bag play, if started) and clean up the
# scratch params file on exit, even if a step below fails - otherwise (with
# `set -e`) this script would abort before reaching this cleanup, leaving
# processes running as orphans that keep stdout (and any pipe/tee reading
# it, e.g. from sync_and_verify_ptl.sh) open forever. RVIZ_PID/BAG_PID are
# pre-declared above so an interruption before they're assigned doesn't
# trip `set -u` here. Goes through ptl_stop (sudo-aware kill + bounded
# wait + SIGKILL escalation) rather than a plain `kill`/`wait` pair -
# fast_livo2 and bag play run as root (see ptl_wrap above), and a plain
# `kill` from this unprivileged script silently fails on them (EPERM),
# which used to leave the trailing `wait` blocked forever.
cleanup() {
  ptl_stop "${LIVO_PID}"
  ptl_stop "${RVIZ_PID}"
  ptl_stop "${BAG_PID}"
  rm -f "${PARAMS_FILE}"
}
trap cleanup EXIT

if [[ "${USE_RVIZ}" == "true" ]]; then
  echo "==> Launching rviz2"
  # No SCHED_FIFO here (rt=0): rviz2 is GUI/rendering work off the timing
  # -critical path; priority-85 FIFO on a process that can block on GL/X11
  # calls risks starving other tasks on its core. taskset pinning alone is
  # enough to keep it off the algorithm's isolated cores, and it isn't
  # launched via sudo/chrt so it doesn't run as root.
  ptl_wrap "${CPUSET_RVIZ}" 0 rviz2 -d "${FASTLIVO2_SRC}/rviz_cfg/ntu_viral.rviz"
  RVIZ_PID="${PTL_LAST_PID}"
fi

sleep 5
echo "==> Playing back ${BAG_ROS2}"
ptl_wrap "${CPUSET_BAG}" 1 ros2 bag play "${BAG_ROS2}"
BAG_PID="${PTL_LAST_PID}"
wait "${BAG_PID}"

echo "==> Bag playback finished, stopping fast_livo2"

if [[ -s "${RESULT_FILE}" ]]; then
  echo "==> Trajectory written to ${RESULT_FILE}"
else
  echo "No trajectory written to ${RESULT_FILE} - check the fast_livo2 log output above." >&2
  exit 1
fi

#!/usr/bin/env bash
# LIMO Pro 자율주행 스택 원샷 실행
#   teleop(base+lidar+EKF+camera) -> Nav2(+rviz) -> 초기 위치 발행
#
# 사용법
#   ./run_lab.sh --pose 3.65 12.20 90      # x y yaw(도).  yaw 90 = +y(복도) 방향
#   ./run_lab.sh --preset room             # 아래 PRESET 표 참고
#   ./run_lab.sh --stop                    # 전부 종료
#   ./run_lab.sh --status                  # 지금 상태만 확인
#
# 초기 위치를 모르면 --pose 없이 실행한 뒤, 화면에 나오는 scan_locate 안내를 따를 것.

set -u

ROS_DISTRO_SETUP=/opt/ros/humble/setup.bash
WS_SETUP="$HOME/wego_ws/install/setup.bash"
LOGDIR="$HOME/lab_logs"
PIDFILE="$HOME/.run_lab.pids"

export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=1
export DISPLAY="${DISPLAY:-:1}"          # rviz는 사용자 X 세션(:1)에 뜬다
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"

# ── 자주 쓰는 시작 위치 (full_v7 좌표계) ───────────────────────────────
# 이름         x      y      yaw(도)   설명
PRESETS="
room        3.65   12.20    90    강의실 개구부 앞, 북향
hall        6.50   12.80   -90    복도 진입 직후, 남향
hall_mid    6.90    8.00   -90    복도 중간, 남향
"

C_OK=$'\e[32m'; C_ERR=$'\e[31m'; C_WARN=$'\e[33m'; C_OFF=$'\e[0m'
say()  { printf '%s\n' "$*"; }
ok()   { printf '%s✔ %s%s\n' "$C_OK"  "$*" "$C_OFF"; }
err()  { printf '%sX %s%s\n' "$C_ERR" "$*" "$C_OFF"; }
warn() { printf '%s! %s%s\n' "$C_WARN" "$*" "$C_OFF"; }
hr()   { printf '%s\n' "────────────────────────────────────────────────────"; }

source_ros() {
  # shellcheck disable=SC1090
  source "$ROS_DISTRO_SETUP"
  if [ ! -f "$WS_SETUP" ]; then err "워크스페이스가 없다: $WS_SETUP"; exit 1; fi
  # shellcheck disable=SC1090
  source "$WS_SETUP"
}

check_env() {
  if [ -n "${CONDA_PREFIX:-}" ]; then
    err "conda 환경이 활성화돼 있다 ($CONDA_PREFIX). ros2가 깨진다."
    err "로그인 메뉴에서 1) Global Environment 를 골라 다시 들어올 것."
    exit 1
  fi
}

# ── 종료 ──────────────────────────────────────────────────────────────
stop_all() {
  hr; say "스택 종료"
  if [ -f "$PIDFILE" ]; then
    while read -r pid name; do
      [ -z "${pid:-}" ] && continue
      if kill -0 "$pid" 2>/dev/null; then
        kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
        ok "SIGINT -> $name (pgid $pid)"
      fi
    done < "$PIDFILE"
    sleep 3
    while read -r pid name; do
      [ -z "${pid:-}" ] && continue
      if kill -0 "$pid" 2>/dev/null; then
        kill -9 -- "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
        warn "SIGKILL -> $name"
      fi
    done < "$PIDFILE"
    rm -f "$PIDFILE"
  else
    warn "$PIDFILE 이 없다. 이름으로 찾아서 정리한다."
  fi
  pkill -f 'navigation_diff_launch|teleop_launch|nav2_container|rviz2' 2>/dev/null
  sleep 1
  ok "종료 완료"
}

# ── 상태 ──────────────────────────────────────────────────────────────
status() {
  source_ros
  hr; say "현재 상태"
  local nodes; nodes=$(timeout 8 ros2 node list 2>/dev/null)
  if [ -z "$nodes" ]; then err "떠 있는 노드 없음"; return; fi

  say "떠 있는 노드 전부:"
  printf '%s\n' "$nodes" | sed 's/^/  /'
  say ""
  say "Nav2 핵심 노드:"
  for n in /map_server /amcl /planner_server /controller_server /bt_navigator /behavior_server; do
    if printf '%s\n' "$nodes" | grep -qx "$n"; then ok "$n"; else err "$n 없음"; fi
  done
  say ""
  say "lifecycle:"
  for n in map_server amcl planner_server controller_server bt_navigator; do
    printf '  %-20s %s\n' "$n" "$(timeout 5 ros2 lifecycle get "/$n" 2>/dev/null || echo '응답 없음')"
  done
  say ""
  say "현재 로봇 자세 (map -> base_link):"
  timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -A3 Translation | head -4 \
    || warn "TF 없음 — AMCL 초기 위치가 아직 안 잡혔다"
}

# ── 진단 덤프 (문제 생기면 이거 한 줄 돌리고 Claude 에게 알릴 것) ──────
diag() {
  source_ros
  local out="$LOGDIR/diag_$(date +%Y%m%d_%H%M%S).txt"
  mkdir -p "$LOGDIR"
  {
    echo "=== date ==="; date
    echo; echo "=== env ==="
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY DISPLAY=$DISPLAY"
    echo; echo "=== node list ==="; timeout 10 ros2 node list 2>&1
    echo; echo "=== lifecycle ==="
    for n in map_server amcl planner_server controller_server bt_navigator behavior_server; do
      echo "$n: $(timeout 5 ros2 lifecycle get "/$n" 2>&1)"
    done
    echo; echo "=== tf map->base_link ==="
    timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -12
    echo; echo "=== /scan 1장 (헤더만) ==="
    timeout 8 ros2 topic echo /scan --qos-reliability best_effort --once 2>&1 | head -12
    echo; echo "=== 최근 nav2 로그의 ERROR/WARN ==="
    grep -nE '\[ERROR\]|\[WARN\]|Exception|failed|Failed' "$(ls -t "$LOGDIR"/nav2_*.log 2>/dev/null | head -1)" 2>/dev/null | tail -80
    echo; echo "=== 최근 teleop 로그의 ERROR/WARN ==="
    grep -nE '\[ERROR\]|\[WARN\]|Exception|failed|Failed' "$(ls -t "$LOGDIR"/teleop_*.log 2>/dev/null | head -1)" 2>/dev/null | tail -40
  } > "$out" 2>&1
  ok "진단 덤프 저장: $out"
  say "Claude 에게 '진단 파일 봐줘' 라고만 하면 된다."
}

# ── 대기 헬퍼 ─────────────────────────────────────────────────────────
wait_for_node() {   # $1=노드이름 $2=제한초 $3=설명
  local t=0
  while [ "$t" -lt "$2" ]; do
    if timeout 5 ros2 node list 2>/dev/null | grep -qx "$1"; then ok "$3 ($1)"; return 0; fi
    sleep 2; t=$((t+2)); printf '.'
  done
  printf '\n'; err "$3 대기 시간 초과 (${2}s) — $1 이 안 뜬다"; return 1
}

wait_for_scan() {
  local t=0
  while [ "$t" -lt 40 ]; do
    if timeout 6 ros2 topic echo /scan --qos-reliability best_effort --once >/dev/null 2>&1; then
      ok "/scan 수신 확인 (라이다 정상)"; return 0
    fi
    sleep 2; t=$((t+2)); printf '.'
  done
  printf '\n'; err "/scan 이 안 온다 — 라이다(/dev/ydlidar) 확인 필요"; return 1
}

wait_for_odom() {
  local t=0
  while [ "$t" -lt 40 ]; do
    if timeout 6 ros2 topic echo /odometry/filtered --once >/dev/null 2>&1; then
      ok "/odometry/filtered 수신 확인 (EKF 정상 = odom->base_link TF 발행 중)"; return 0
    fi
    sleep 2; t=$((t+2)); printf '.'
  done
  printf '\n'; err "EKF 출력이 없다 — 이게 없으면 Nav2 costmap이 TF 대기로 멈춘다"; return 1
}

# ── 초기 위치 ─────────────────────────────────────────────────────────
publish_pose() {   # $1=x $2=y $3=yaw(도)
  local qz qw
  read -r qz qw < <(python3 -c "
import math
a = math.radians($3) / 2.0
print('%.6f %.6f' % (math.sin(a), math.cos(a)))
")
  say "초기 위치 발행: x=$1 y=$2 yaw=$3° (qz=$qz qw=$qw)"
  # --once 는 디스커버리 완료 전에 쏘고 끝나 AMCL(BEST_EFFORT 구독)이 놓친다.
  # -w 1 --times 3 이 필수.
  ros2 topic pub -w 1 --times 3 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $1, y: $2, z: 0.0}, \
      orientation: {z: $qz, w: $qw}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, \
      0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}" >/dev/null 2>&1
  ok "초기 위치 발행 완료 (3회)"
}

locate_help() {
  hr
  warn "초기 위치를 안 줬다. AMCL이 map->odom 을 발행하기 전에는"
  warn "global costmap 구성이 막혀서 bt_navigator 가 inactive 에 머문다."
  say ""
  say "위치를 모르면 스캔 한 장으로 역산할 것:"
  say ""
  say "  # 로봇에서"
  say "  ros2 topic echo /scan --qos-reliability best_effort --once --full-length > ~/scan.yaml"
  say ""
  say "  # 맥북에서"
  say "  scp wego@192.168.0.96:~/scan.yaml ~/limo-lab/merge/"
  say "  cd ~/limo-lab/merge"
  say "  python3 scan_locate.py full_v7.yaml scan.yaml --xmin -4.5 --xmax 5.0 --ymin 0 --ymax 16"
  say ""
  say "  score 0.8 이상이면 신뢰 가능. *_match.png 로 눈으로도 확인할 것."
  say "  나온 좌표로:  ./run_lab.sh --pose <x> <y> <yaw>"
  say ""
  say "rviz 의 2D Pose Estimate 로 찍어도 된다 (덜 정확하지만 빠름)."
  hr
}

# ── 인자 파싱 ─────────────────────────────────────────────────────────
POSE_X=""; POSE_Y=""; POSE_YAW=""
case "${1:-}" in
  --stop)   stop_all; exit 0 ;;
  --status) status;   exit 0 ;;
  --diag)   diag;     exit 0 ;;
  --pose)
    POSE_X="${2:-}"; POSE_Y="${3:-}"; POSE_YAW="${4:-}"
    if [ -z "$POSE_YAW" ]; then err "사용법: --pose <x> <y> <yaw도>"; exit 1; fi ;;
  --preset)
    name="${2:-}"
    line=$(printf '%s\n' "$PRESETS" | awk -v n="$name" '$1==n {print; exit}')
    if [ -z "$line" ]; then
      err "그런 preset 없다: '$name'"; say "쓸 수 있는 것:"; printf '%s\n' "$PRESETS" | sed '/^$/d' | sed 's/^/  /'
      exit 1
    fi
    POSE_X=$(echo "$line" | awk '{print $2}')
    POSE_Y=$(echo "$line" | awk '{print $3}')
    POSE_YAW=$(echo "$line" | awk '{print $4}') ;;
  "" ) ;;
  * ) err "모르는 옵션: $1"; exit 1 ;;
esac

# ── 본 실행 ───────────────────────────────────────────────────────────
check_env
source_ros
mkdir -p "$LOGDIR"
TS=$(date +%Y%m%d_%H%M%S)
TELEOP_LOG="$LOGDIR/teleop_$TS.log"
NAV_LOG="$LOGDIR/nav2_$TS.log"
: > "$PIDFILE"

if timeout 8 ros2 node list 2>/dev/null | grep -qE '^/(amcl|ydlidar_ros2_driver_node|limo_base_node)$'; then
  err "스택이 이미 떠 있다. 먼저 ./run_lab.sh --stop 으로 정리할 것."
  exit 1
fi

hr
say "LIMO 자율주행 스택 기동   맵: full_v7   ROS_DOMAIN_ID=$ROS_DOMAIN_ID   DISPLAY=$DISPLAY"
hr

say "[1/4] teleop 스택 (base + lidar + EKF + camera)"
setsid nohup ros2 launch wego teleop_launch.py > "$TELEOP_LOG" 2>&1 < /dev/null &
echo "$! teleop" >> "$PIDFILE"
say "      로그: $TELEOP_LOG"
say "      (노드 이름 대신 실제 데이터가 오는지로 판단한다)"
wait_for_scan || { err "teleop 로그 확인 필요: $TELEOP_LOG"; exit 1; }
wait_for_odom || { err "teleop 로그 확인 필요: $TELEOP_LOG"; exit 1; }

say ""
say "[2/4] Nav2 (map_server + amcl + planner + controller + rviz)"
setsid nohup ros2 launch wego navigation_diff_launch.py > "$NAV_LOG" 2>&1 < /dev/null &
echo "$! nav2" >> "$PIDFILE"
say "      로그: $NAV_LOG"
wait_for_node /amcl 60 "AMCL 기동" || { err "nav2 로그를 볼 것: $NAV_LOG"; exit 1; }
wait_for_node /planner_server 30 "planner 기동" || true
sleep 3

say ""
say "[3/4] 초기 위치"
if [ -n "$POSE_YAW" ]; then
  publish_pose "$POSE_X" "$POSE_Y" "$POSE_YAW"
  sleep 3
else
  locate_help
fi

say ""
say "[4/4] 상태 확인"
status

hr
if [ -n "$POSE_YAW" ]; then
  say "rviz 에서 라이다 스캔이 맵의 벽과 겹치는지 반드시 눈으로 확인할 것."
  say "sigma 숫자는 '파티클이 모였나'일 뿐 '실제와 맞나'가 아니다."
  say ""
  say "안 맞으면:  rviz 의 2D Pose Estimate, 또는"
  say "            ./run_lab.sh --pose <x> <y> <yaw>  (스택 켜진 채로 다시 실행하면 거부되니"
  say "            초기 위치만 다시 줄 때는 아래 명령을 직접 쓸 것)"
  say ""
  say '  ros2 topic pub -w 1 --times 3 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \'
  say '    "{header: {frame_id: '"'"'map'"'"'}, pose: {pose: {position: {x: X, y: Y, z: 0.0}, ...}}}"'
fi
say ""
say "goal 은 토픽 말고 action 으로 (토픽은 디스커버리 전에 쏘고 끝나 유실된다):"
say '  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \'
say '    "{pose: {header: {frame_id: '"'"'map'"'"'}, pose: {position: {x: 6.50, y: 12.80, z: 0.0}, orientation: {z: -0.7071, w: 0.7071}}}}"'
say ""
say "종료: ./run_lab.sh --stop        상태: ./run_lab.sh --status"
say "로그: tail -f $NAV_LOG"
hr

#!/usr/bin/env bash
# start_relay.sh —— 中转端启动脚本（video_subscriber_h264）
#   订阅上位机的 H.264 CompressedImage -> popen 喂 GStreamer -> 转 RTP/UDP 到本机 5600
#   YOLO 端（rknn_yolo_detector）从 udpsrc 127.0.0.1:5600 取流
#
# 与 Foxy 版的差别：ROS 发行版不再写死，按 /opt/ros/<distro> 自动探测。
# 可用环境变量覆盖：TOPIC / MAX_QUEUE / UDP_PORT / SHOW
#
# 用法： bash ~/videos_ws/start_relay.sh

# ---- 1. ROS 2 环境：自动探测发行版 ----------------------------------------
if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
else
  ROS_SETUP=""
  for d in humble jazzy iron foxy galactic; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
      ROS_SETUP="/opt/ros/$d/setup.bash"
      break
    fi
  done
fi

if [ -z "$ROS_SETUP" ]; then
  echo "[ERR] 找不到任何 /opt/ros/*/setup.bash，ROS 2 未安装？" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
echo "[ok] ROS 环境: $ROS_DISTRO  ($ROS_SETUP)"

# ---- 2. 工作空间 overlay ----------------------------------------------------
WS="${WS:-$HOME/videos_ws}"
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  echo "[ok] 工作空间: $WS"
else
  echo "[warn] $WS/install/setup.bash 不存在，先在 $WS 里 colcon build" >&2
fi

# ---- 3. conda 抢占 python3 的自检（只提示，不改环境） ----------------------
if command -v python3 >/dev/null 2>&1; then
  PY3="$(command -v python3)"
  if [ "$PY3" != "/usr/bin/python3" ]; then
    echo "[warn] python3 指向 $PY3（ROS 2 CLI 可能异常，建议 conda deactivate）" >&2
  fi
fi

# ---- 4. 参数（与 readme.md 第 5 节保持一致） -------------------------------
TOPIC="${TOPIC:-/drone1/camera_rgb/image_raw/compressed}"
MAX_QUEUE="${MAX_QUEUE:-90}"
UDP_PORT="${UDP_PORT:-5600}"
# 注意：节点里的 show 参数实际含义是「启动 GStreamer sink 管道」，
#       中转给推理端时也必须为 true，不是单纯控制有没有画面。
SHOW="${SHOW:-true}"

# sink_cmd 是整条 shell 命令（节点用 popen 执行），必须带 gst-launch-1.0
SINK_CMD="gst-launch-1.0 fdsrc fd=0 blocksize=65536 ! \
queue max-size-buffers=0 max-size-bytes=4000000 max-size-time=0 ! \
h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=${UDP_PORT}"

# ---- 5. GStreamer 依赖自检（缺元件就直接失败，别等黑屏） -------------------
if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo "[ERR] 没有 gst-launch-1.0，先装 gstreamer1.0-tools" >&2
  exit 1
fi
MISSING=""
for el in fdsrc queue h264parse rtph264pay udpsink; do
  gst-inspect-1.0 "$el" >/dev/null 2>&1 || MISSING="$MISSING $el"
done
if [ -n "$MISSING" ]; then
  echo "[ERR] 缺少 GStreamer 元件:$MISSING" >&2
  echo "      插件包：gstreamer1.0-plugins-base(-good) 提供 h264parse/rtph264pay/udpsink" >&2
  exit 1
fi

# ---- 6. 启动 ---------------------------------------------------------------
echo "[run ] $TOPIC  ->  RTP 127.0.0.1:${UDP_PORT}  | max_queue=$MAX_QUEUE | sink=$SHOW"
echo "[tip ] 上游是否在发，用另一个终端看： ros2 topic hz $TOPIC"
exec ros2 run video_subscriber_cpp videoSubscriberH264 --ros-args \
  -p topic:="$TOPIC" \
  -p show:="$SHOW" \
  -p max_queue:="$MAX_QUEUE" \
  -p sink_cmd:="$SINK_CMD"

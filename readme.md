# 视频流订阅与 YOLO 识别链路 —— 状态备份

- 初版备份：2026-10-21（Ubuntu 20.04 + ROS2 Foxy 时代）
- **本次修订：2026-11-24**（系统已重装为 Ubuntu 22.04 + ROS2 Humble，原文档多处失效）
- 当前状态：稳定运行（1080p H.264 视频流 + YOLO 推理 + 串口输出）
- 适用平台：飞凌 OK3588 开发板 / Ubuntu 22.04 / 内核 5.10.209 / ROS2 Humble

> 10-21 那版描述的是 Foxy + `avdec_h264` + `192.168.0.6` 的组合，**整套已经不可用**。
> 本文档以 2026-11-22 交接文档为环境基线，叠加 11-22 至 11-24 的实测修复记录。

---

## 1. 系统架构

    上位机 (Fast DDS, 192.168.0.101)
       |  /drone1/camera_rgb/image_raw/compressed
       |  (sensor_msgs/CompressedImage, format=h264, Annex-B, 一帧一条消息)
       v
    [中转端] video_subscriber_h264  (~/videos_ws, video_subscriber_cpp 包)
       |  订阅 -> 等待首个 IDR -> 送显管道
       |  sink_cmd: gst-launch-1.0 fdsrc ! queue ! h264parse ! rtph264pay
       |            ! udpsink 127.0.0.1:5600
       |  同时回传 /video/frame_ack (std_msgs/Header, 测 RTT 用，上位机可不订阅)
       v
    [YOLO 端] rknn_yolo_detector  (~/rknn_yolo_project)
       |  video_source: udpsrc 5600 -> rtpjitterbuffer -> rtph264depay -> h264parse
       |               -> mppvideodec -> videoconvert -> appsink
       |  OpenCV 取帧 -> NPU 推理 (best_0722_fp.rknn, 640x640 输入)
       |  -> 画框 -> 显示窗口 "RKNN YOLO Detector"
       +-> 串口输出 /dev/ttyS9 @ 115200 (字符模式 @DET 行 / 字节流模式 21 字节包)

- **网络拓扑**：`eth0` = `192.168.137.2/24`（Windows ICS 直连，SSH 通道）；
  `eth1` = `192.168.0.103/24`（ROS 网段，接交换机，对端 `192.168.0.101`）。
  两个口都是 `never-default`，**当前无默认路由**，`apt`/`pip` 需临时借道
  `sudo ip route add default via 192.168.0.1 dev eth1`。
  > 10-21 备份里记的 `eth1 = 192.168.0.6` 是重装前的旧地址，已作废。
- **视频参数**：1920x1080，源头 25~30fps，H.264 Annex-B，IDR 间隔约 1.2s 且自带
  SPS/PPS（`types=[7,8,5,...]`），一帧约 16 个 slice。

## 2. 目录与关键文件

| 路径 | 说明 |
|---|---|
| `~/videos_ws/src/video_subscriber_cpp/` | 中转端 ROS2 包（`videoSubscriberH264.cpp` 为当前链路） |
| `~/videos_ws/start_relay.sh` | 中转端一键启动脚本（**已纳入 git**，见问题 #5） |
| `~/videos_ws/protocol/` | 控制帧协议 v1（`protocol.md` + `frame_codec.h` + MATLAB 侧） |
| `~/rknn_yolo_project/` | YOLO 推理工程（`src/yolo_main.cpp`、`opencv_frame_source.cpp` 等） |
| `~/rknn_yolo_project/detector_config.txt` | YOLO 端配置（运行时读取，改完不用重编） |
| `~/rknn_yolo_project/models/` | `best_0722_fp.rknn` + `.names.txt` —— **仅存于板子，WSL 无副本** |
| `~/rknn_yolo_project/third_party/` | opencv / rknn runtime / GStreamer 旧闭包 —— **仅存于板子** |
| `~/rknn_yolo_project/build/rknn_yolo_detector` | 已编好的可执行文件，**重刷后未必能复现，务必备份** |
| `~/rknn_yolo_project/scripts/run_yolo.sh` | YOLO 端正式启动脚本 |
| `~/rknn_yolo_project/scripts/run_yolo_test.sh` | 插件加载策略诊断（`PLUGIN_MODE=bundled/both/system`） |
| `~/rknn_yolo_project/scripts/serial_probe.sh` | 串口出波形诊断（区分程序未写 / 内核未发 / 引脚未出） |
| `~/check_sync.sh` | 板上源码与 WSL 快照逐字节比对（md5 清单已内嵌） |
| `~/.bashrc` | 仅 `source /opt/ros/humble/setup.bash`（第 120 行） |
| `~/fastrtps_profile.xml` | **保留但已不加载**，见第 3 节 |

> WSL 侧只有 `src/`、`include/`、`scripts/`、`CMakeLists.txt`、`detector_config.txt`
> 约 100KB 源码，**不含 `models/`、`third_party/`、`build/`**，无法独立重建。
> 备份必须以板子为准。

## 3. 环境配置（当前生效状态）

**ROS/DDS 走默认多播，没有任何 profile 覆盖。** `.bashrc` 里与 ROS 相关的实际内容：

    source /opt/ros/humble/setup.bash              # 第 120 行（fishros 安装时写入）
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp     # 第 138 / 143 行 —— 重复两遍
    export ROS_DOMAIN_ID=0                         # 第 139 / 144 行 —— 重复两遍
    # export FASTRTPS_DEFAULT_PROFILES_FILE=...    # 第 140 / 145 行 —— 已注释

- RMW：`rmw_fastrtps_cpp`，FastDDS 2.6.12。**注意这是显式 export 的**——虽然它同时
  也是 Humble 的默认值，但换 RMW 时别以为改系统默认就够，这里写死了
- `ROS_DOMAIN_ID`：`0`，显式 export，必须与对端一致
- `ROS_LOCALHOST_ONLY`：`0`，即不限制本机通信，正常
- ⚠️ **138-140 与 143-145 是内容重复的两段**（历史叠加所致）。值相同所以当前无害，
  但将来要改就得两处一起改——这是 `.bashrc` 里最容易踩空的地方
- ⚠️ 上述第 140 / 145 行曾强制 `export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastrtps_profile.xml`，
  **现已注释**。那个 profile 把 `metatrafficUnicastLocatorList` 写死旧 IP `192.168.0.6`、
  `initialPeersList` 只指向 `0.101` 且关多播，还带 `is_default_profile="true"` 全局覆盖。
  表现是「进程正常 publish、谁都看不见、不报错」。
  **这两行是历史根因，绝对不要重新打开**；`~/fastrtps_profile.xml` 保留未删，
  仅供将来遇到「交换机禁多播」的环境时改地址复用。

**GStreamer（实测 2026-11-24）**：核心 `1.20.3`，`gstreamer1.0-rockchip1 1.14`
提供硬解 `mppvideodec`。以下元件**全部存在**，这条链路的依赖是完整的：

    h264parse  rtph264depay  mppvideodec  udpsink  videoconvert  appsink

`avdec_h264` **不存在**（未安装 `gstreamer1.0-libav`），也不需要——`mppvideodec`
走 MPP 硬件解码，比软解省 CPU，且正是问题 #7 的解法。

**RKNN 运行时**：系统 `/usr/lib/librknnrt.so` 是 `1.3.0 (2022-05-04)`，偏旧；
但推理二进制通过 RPATH 链接的是工程自带的
`third_party/rknn/runtime/Linux/librknn_api/aarch64/librknnrt.so`，
系统那份不参与，升级/降级都不影响。`dpkg -l | grep rknpu` 查不到任何东西属正常
——`rknpu` 驱动编在内核里（`dmesg` 可见 `RKNPU fdab0000.npu`），不是发行版包。

**内核 UDP 缓冲**：`net.core.rmem_max` 的 8MB 设置在重装后**已丢失**，
实测当前值 `sysctl net.core.rmem_max` = **212992**（208KB，系统默认）。表现为：

    gst-launch-1.0 ... udpsrc buffer-size=2097152 ...
    WARNING: Could not create a buffer of requested 2097152 bytes
             (Operation not permitted). Need net.admin privilege?

实际退化为默认 208KB。当前不致命（中转端与检测器同机，走 loopback，且队列已兜住
IDR 突发），但**一旦出现画面糊块，第一个要查这里**。恢复方法（需改系统配置，按需执行）：

    net.core.rmem_max=8388608
    net.core.rmem_default=8388608

## 4. 启动流程（3 步）

    # 0) 前置确认：上位机发布节点已启动
    ping 192.168.0.101
    ros2 topic list -t | grep drone1     # 期望: [sensor_msgs/msg/CompressedImage]

    # 1) 中转端（终端 1）
    bash ~/videos_ws/start_relay.sh

    # 2) YOLO 端（终端 2）—— 必须带 PLUGIN_MODE=system
    cd ~/rknn_yolo_project && PLUGIN_MODE=system bash scripts/run_yolo_test.sh

    # 3) 串口对端发 0x11 切字节流模式（HEX 方式发，不是文本模式）

正常日志标志：

- 中转端：`[ok] ROS 环境: humble` → `订阅: /drone1/camera_rgb/image_raw/compressed`
  → `已等到首个 IDR，开始送显` → `Setting pipeline to PLAYING`
  → 统计行 `约 27 fps | 帧间隔 37 ms | 显示积压 1 帧 | 丢帧 0`
- YOLO 端：`[OpenCV][debug] ... actual=1920x1080` → 黑屏约 1.5~2s（等 IDR）
  → 出干净画面 → `[DET] frame=N` 持续（FPS 约 13.8）→ 无 `[WATCHDOG]`
- 串口：发 `0x11` 后终端打印 `[SERIAL] 输出模式切换为：字节流`，
  此后每帧 21 字节（约 13.8 次/秒）

## 5. 关键配置参数

`detector_config.txt`（YOLO 端，当前生效值）：

    model_path=models/best_0722_fp.rknn
    labelName_path=models/best_0722.names.txt
    video_source=udpsrc uri=udp://127.0.0.1:5600 buffer-size=2097152 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtpjitterbuffer latency=200 ! rtph264depay ! h264parse ! mppvideodec ! videoconvert ! appsink sync=false
    camera_backend=gstreamer
    camera_width=1920        # 管道源时被忽略（opencv_frame_source.cpp 有 is_pipeline 保护）
    camera_height=1080       # 同上，改多少都不生效，别指望它
    camera_fps=0
    confidence_threshold=0.25
    nms_threshold=0.45
    yolo_thread_num=1
    yolo_core_sequence=auto
    no_show=0                # 0 需显示环境；SSH/无 DISPLAY 时改 1
    debug=1                  # 每 30 帧打一条状态
    serial_port=/dev/ttyS9
    serial_baud=115200
    serial_interval=1        # 每帧触发串口输出

`start_relay.sh` 启动参数（环境变量可覆盖）：

- `TOPIC` = `/drone1/camera_rgb/image_raw/compressed`
- `MAX_QUEUE` = 90（显示缓存帧数；出现「显示消费过慢」时降到 30）
- `UDP_PORT` = 5600
- `SHOW` = true —— **注意语义：它表示「是否启动 GStreamer sink 管道」，不是「弹不弹窗」。
  中转给推理端时必须是 true，设 false 会导致整条链路一个字节都不转发。**

## 6. 历史问题与修复记录（备份重点，勿回退）

| # | 现象 | 根因 | 修复位置 |
|---|---|---|---|
| 1 | 显示黑屏只有 YOLO 框、无画面 | `OpenCvFrameSource::open()` 对 GStreamer 管道源无条件执行 `cap_.set(WIDTH/HEIGHT/FPS)`，破坏 caps 协商，`cap_.read()` 永远取不到帧 | `opencv_frame_source.cpp`：管道源（`video_source` 含 `!`）跳过全部 set；非管道源加 `>0` 保护 |
| 2 | 画面正常但 5 秒误报 `[WATCHDOG]` 强退 | `g_last_frame_ms` 刷新语句被误写进 `frame.empty()` 失败分支 | `yolo_main.cpp` `captureLoop`：刷新移到取帧成功路径 |
| 3 | 启动前 2 秒拖影/底部糊块 | 中途加入缺参考帧（等 IDR 约 1.2s）+ UDP 接收缓冲溢出丢包（IDR 约 130 包突发） | 中转端 `onFrame` 加「等 IDR 再送显」门控；`udpsrc buffer-size=2097152`；内核 rmem 8MB；`output-corrupt=false`；中转端队列溢出改为整体丢弃+重新等 IDR |
| 4 | 与上位机 DDS 互相发现不了 | 多网卡下 CycloneDDS 0.7 绑定接口 Segfault、选错网卡；组播被 profile 里的静态 locator 破坏 | 改用默认多播；**注释掉 `.bashrc` 里的 `FASTRTPS_DEFAULT_PROFILES_FILE` 两行**（见第 3 节） |
| 5 | `start_relay.sh` 报 `/opt/ros/foxy/setup.bash: No such file or directory`，脚本静默终止 | 脚本是 Foxy 时代写的，写死 `/opt/ros/foxy`；且**从未纳入 git，只在板子上** | `start_relay.sh` 重写：优先用 `$ROS_DISTRO`，否则按 humble→jazzy→iron→foxy 探测；自动 source overlay；上游预检只告警不阻塞 |
| 6 | YOLO 端 `sh: 1: fdsrc: not found`，节点日志一切正常但无画面 | `sink_cmd` 交给 `popen()`，等价于 `sh -c "<cmd>"`，必须以 `gst-launch-1.0` 开头 | `start_relay.sh` 的 `SINK_CMD` 补全命令前缀，并加 4MB `queue` 吸收 IDR 突发 |
| 7 | YOLO 端 `no element "h264parse"`，`libgstvideoparsersbad.so` 报 `undefined symbol: g_memdup2` | 工程自带 `third_party/runtime` 是 20.04 的 GStreamer 1.16 + GLib 2.64 闭包，被 RPATH 优先加载；系统插件（1.20 编译）core 版本高于 1.16，加载器直接拒绝 | 放弃自带闭包，整体走系统栈：`LD_PRELOAD` 系统 gst/glib 全家桶 + 系统插件目录，解码器换 `mppvideodec` |
| 8 | 上述方案仍崩：`GLIBCXX_3.4.29 not found (required by librga.so.2)` | `LD_PRELOAD` 只换了 gst/glib，`libstdc++` 仍被 RPATH 拽到 20.04 那份（GCC 9，最高 GLIBCXX_3.4.28） | 补充预加载系统 `libstdc++.so.6` / `libselinux.so.1` / `libz.so.1`；根治方案是去掉 `CMakeLists.txt` 中 `BUNDLED_RPATH` 的 `third_party/runtime/aarch64` 段并重编 |
| 9 | 串口助手收不到任何数据、示波器无波形 | 字符模式下 `writeSerialDetection()` 遇到 `results.empty()` 直接 return —— **画面里没有 person/dog/cat 时一个字节都不发，且不报错** | 对端发 `0x11` 切字节流模式，此后空帧也发 21 字节心跳包 |

严禁回退的改动：#1 的 set 保护、#2 的刷新位置、#3 的 IDR 门控与 buffer-size、
#4 的 profile 注释、#5 的发行版探测、#6 的 `gst-launch-1.0` 前缀、#7/#8 的系统栈策略。
回退任一项都会复现对应故障。

## 7. 已知良性告警（见到可忽略）

- `Failed to load plugin .../third_party/runtime/.../libgst*.so: libXXX.so.N: cannot open
  shared object file`（`libdc1394` `libaom` `libx265` `libmms` `libsrt` `libvpx`
  `libwebp` `libfluidsynth` `libdvdread` `libusrsctp` `libofa` `libIlmImf` `libnettle`
  等十余条）：自带 20.04 闭包缺配套库，插件扫描期失败，**链路完全不用这些插件**。
- `libselinux.so.1: no version information available`：版本号缺失但接口兼容，非致命。
- `Cannot query video position ... duration=-1`：UDP 直播流无时长，OpenCV 转发的 warning。
- `getProperty ... unhandled property: 6`：查询 FOURCC 不被 GStreamer 后端支持。
- `AT-SPI: Error retrieving accessibility bus address`：GTK 无障碍服务缺失。
- `Could not create a buffer of requested 2097152 bytes`：见第 3 节 rmem_max。

**判真原则**：出现 `no element "XXX"` 或 `undefined symbol` 才是真故障；
只要「已等到首个 IDR」和 `[DET]` 持续输出，上述 WARN 均可无视。
真正该盯的异常信号是：`[WATCHDOG]`、`读取到空图像`、`打开视频输入源失败`。

## 8. 验证与排障速查

    # 帧率 / 格式
    ros2 topic hz /drone1/camera_rgb/image_raw/compressed           # 对端 25~30
    ros2 topic hz /video/frame_ack                                  # 中转端出帧，约 27
    ros2 topic echo /drone1/camera_rgb/image_raw/compressed | head -20
    # 期望 format: h264，data 开头 0 0 0 1 ...

    # 丢包（跑几分钟前后各看一次，增量应为 0 或个位数）
    nstat -az | grep UdpRcvbufErrors

    # DDS 发现不了话题
    ros2 daemon stop && ros2 topic list -t
    ros2 topic list --no-daemon                                      # 绕开 daemon 缓存
    ping 192.168.0.101

    # 串口：程序到底有没有在写（决定性判据）
    watch -n1 "grep -E '^ *9: uart' /proc/tty/driver/serial"
    # tx 计数每秒涨 ~21 => 程序在写；一直不动 => 没写（多半是字符模式且无目标）

故障速判：

1. 画面拖影/糊块 → 查 `UdpRcvbufErrors` 增量；必要时恢复 rmem_max 8MB
2. 5 秒 WATCHDOG 强退 → 先看中转端统计行是否停（上游断流）vs `ros2 topic hz` 是否还在
3. 黑屏但无 WATCHDOG → 等 IDR 窗口（约 1.2s）内属正常，超 3s 查中转端日志
4. `no element "XXX"` → 插件加载失败，跑 `PLUGIN_MODE=system bash scripts/run_yolo_test.sh`
   看诊断表；`gst-inspect-1.0 XXX` 确认元件是否存在
5. 串口无数据 → ①对端发过 `0x11` 没有 ②`tx` 计数涨不涨 ③`bash scripts/serial_probe.sh`
   看设备树 status 与引脚连通性

## 9. 性能现状

| 指标 | 实测值 |
|---|---|
| 对端发布帧率 | 25~30 fps（可变，曾降到 3Hz） |
| 中转端订阅帧率 | 26.5~27.5 fps，帧间隔 36.5~37.2ms（抖动 <1ms） |
| 中转端显示积压 | 恒为 1 帧（写线程跟得上，不积压） |
| 中转端丢帧 | 0 |
| 推理/显示刷新率 | 约 13.8 fps（单线程推理瓶颈） |
| 输入队列丢帧 | 有（`input_queue` 容量 10、`tryPopLatest` 取最新，设计内降级保低延迟） |
| 串口 | `/dev/ttyS9` 115200：字符模式无目标不发；`0x11` 切字节流后每帧 21 字节（含空帧心跳），`0x10` 切回 |

**注意 27 vs 13.8 的差值不是故障**：中转 27fps 进、推理 13.8fps 出，中间是设计内丢帧。

## 10. 后续待办（不在本次备份范围）

- [ ] **串口三帧协议 v2**（心跳 / 航迹点 / 目标参数）——方案讨论中，未落代码。
      要点：统一帧头、加 seq、校验换 CRC-8、串口写改独立线程。
      当前 21 字节检测帧的 `tick` 恒为 0，接收端无法判断断流，属已知缺陷。
- [ ] 三帧协议落地后，MATLAB / 飞控侧同步适配解析
- [ ] `CMakeLists.txt` 去掉 `BUNDLED_RPATH` 里的 `third_party/runtime/aarch64`，重编后彻底摆脱 `LD_PRELOAD`
- [ ] 恢复内核 `net.core.rmem_max=8388608`
- [ ] 雷达点云订阅 + 障碍物坐标计算 + 避障轨迹串口输出（方案已梳理，未实施）
- [ ] 可选：显示与推理解耦 / 多 NPU 核心并行提升显示帧率（当前不优化）
- [ ] 可选：中转端开机自启 systemd 服务

---

## 附：版本沿革

| 日期 | 变更 |
|---|---|
| 2026-10-21 | 初版。Foxy + `avdec_h264` + 静态 locator + `eth1=192.168.0.6` |
| 2026-11-20 | 重装 Ubuntu 22.04 + Humble，上述配置全部失效 |
| 2026-11-22 | 打通 DDS（注释 profile）、确认 NPU 推理可用 |
| 2026-11-24 | 打通中转端 `start_relay.sh`、切换系统 GStreamer 栈 + `mppvideodec`、确认串口字节流输出、本文档重写 |

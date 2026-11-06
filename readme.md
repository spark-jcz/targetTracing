# 视频流订阅与 YOLO 识别链路 —— 状态备份

- 备份日期：2026-10-21
- 备份状态：稳定运行（1080p@25fps H.264 视频流 + YOLO 推理 + 串口输出）
- 适用平台：飞凌 OK3588 开发板 / Ubuntu 20.04 / ROS2 Foxy

---

## 1. 系统架构

    上位机 (Fast DDS, 192.168.0.101)
       |  /drone1/camera_rgb/image_raw/compressed
       |  (sensor_msgs/CompressedImage, format=h264, Annex-B, 一帧一条消息)
       v
    [中转端] video_subscriber_h264  (~/videos_ws, video_subscriber_cpp 包)
       |  订阅 -> 等待首个 IDR -> 送显管道
       |  sink_cmd: fdsrc ! h264parse ! rtph264pay ! udpsink 127.0.0.1:5600
       |  同时回传 /video/frame_ack (std_msgs/Header, 测 RTT 用，上位机可不订阅)
       v
    [YOLO 端] rknn_yolo_detector  (~/rknn_yolo_project)
       |  video_source: udpsrc 5600 -> rtpjitterbuffer -> rtph264depay -> h264parse
       |               -> avdec_h264(output-corrupt=false) -> videoconvert -> appsink
       |  OpenCV 取帧 -> NPU 推理 (best_0722_fp.rknn, 640x640 输入)
       |  -> 画框 -> 显示窗口 "RKNN YOLO Detector"
       +-> 串口输出 /dev/ttyS9 @ 115200 (字符模式 @DET 行 / 字节流模式 21 字节包)

- 网络拓扑：开发板 eth0=192.168.137.2（SSH 线），eth1=192.168.0.6（与上位机
  192.168.0.101 通信口）。DDS 静态 peer 用的是 192.168.0.6 / 192.168.0.101。
- 视频参数：1920x1080，25fps（源头即 25，非链路丢帧），H.264 Annex-B，
  IDR 间隔约 1.2s 且自带 SPS/PPS（types=[7,8,5,...]），一帧约 16 个 slice。

## 2. 目录与关键文件

| 路径 | 说明 |
|---|---|
| ~/videos_ws/src/video_subscriber_cpp/ | 中转端 ROS2 包（videoSubscriberH264.cpp） |
| ~/videos_ws/start_relay.sh | 中转端一键启动脚本 |
| ~/rknn_yolo_project/ | YOLO 推理工程（src/yolo_main.cpp, opencv_frame_source.cpp 等） |
| ~/rknn_yolo_project/detector_config.txt | YOLO 端配置 |
| ~/rknn_yolo_project/scripts/run_yolo.sh | YOLO 端启动脚本（自动配置 GStreamer 插件环境） |
| ~/fastrtps_profile.xml | DDS 静态单播配置（绑定 192.168.0.6，peer 192.168.0.101） |
| ~/.bashrc | ROS/DDS 环境变量（三行 export） |

## 3. 环境配置（开机后自动生效，来自 ~/.bashrc）

    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=0
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastrtps_profile.xml

~/fastrtps_profile.xml 内容（Fast DDS 静态单播。因 CycloneDDS 0.7 在多网卡
下绑定接口会 Segfault，且组播发现受网络环境影响，最终方案为双端 Fast DDS +
静态 locator）：

    <?xml version="1.0" encoding="UTF-8"?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
      <participant profile_name="unicast" is_default_profile="true">
        <rtps>
          <builtin>
            <metatrafficUnicastLocatorList>
              <locator><udpv4><address>192.168.0.6</address></udpv4></locator>
            </metatrafficUnicastLocatorList>
            <initialPeersList>
              <locator><udpv4><address>192.168.0.101</address></udpv4></locator>
            </initialPeersList>
          </builtin>
        </rtps>
      </participant>
    </profiles>

内核 UDP 缓冲（/etc/sysctl.conf 持久化，吸收 IDR 大包突发，防丢包花屏）：

    net.core.rmem_max=8388608
    net.core.rmem_default=8388608

## 4. 启动流程（3 步）

    # 0) 前置确认：上位机发布节点已启动
    ping 192.168.0.101
    ros2 topic list -t | grep drone1     # 期望: [sensor_msgs/msg/CompressedImage]

    # 1) 中转端（终端 1）
    bash ~/videos_ws/start_relay.sh

    # 2) YOLO 端（终端 2）
    cd ~/rknn_yolo_project && bash scripts/run_yolo.sh

正常日志标志：

- 中转端：订阅: /drone1/camera_rgb/image_raw/compressed
  -> 已等到首个 IDR，开始送显（起始丢弃 N 条非关键帧）
  -> 统计行 25.0 fps | 丢帧 0
- YOLO 端：[OpenCV][debug] ... actual=1920x1080
  -> 黑屏约 1.5~2s（等 IDR + 管道预卷）-> 出干净画面
  -> [DET] frame=N 持续输出（FPS约13.8）-> 无 [WATCHDOG]

## 5. 关键配置参数

detector_config.txt（YOLO 端，节选当前生效值）：

    model_path=models/best_0722_fp.rknn
    labelName_path=models/best_0722.names.txt
    video_source=udpsrc uri=udp://127.0.0.1:5600 buffer-size=2097152 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtpjitterbuffer latency=200 ! rtph264depay ! h264parse ! avdec_h264 output-corrupt=false ! videoconvert ! appsink sync=false
    camera_backend=gstreamer
    camera_width=0          # 管道源不做 set（0 即跳过），勿改回非 0
    camera_height=0
    camera_fps=0
    confidence_threshold=0.25
    nms_threshold=0.45
    yolo_thread_num=1
    yolo_core_sequence=auto
    no_show=0
    debug=0
    serial_port=/dev/ttyS9
    serial_baud=115200
    serial_interval=1       # 每帧触发串口输出；字符模式下无目标不发数据
    stream_timeout_sec=5    # 无帧看门狗，5 秒无新帧判定断流强退

中转端启动参数（start_relay.sh 顶部）：

- topic = /drone1/camera_rgb/image_raw/compressed
- max_queue = 90（显示缓存帧数；出现「显示消费过慢」告警时降为 30）
- sink_cmd = fdsrc ! h264parse ! rtph264pay ! udpsink 127.0.0.1:5600

## 6. 历史问题与修复记录（备份重点，勿回退）

| # | 现象 | 根因 | 修复位置 |
|---|---|---|---|
| 1 | 显示黑屏只有 YOLO 框、无画面 | OpenCvFrameSource::open() 对 GStreamer 管道源无条件执行 cap_.set(WIDTH/HEIGHT/FPS)，破坏 caps 协商，cap_.read() 永远取不到帧 | opencv_frame_source.cpp：管道源（video_source 含 !）跳过全部 set；非管道源加 >0 保护 |
| 2 | 画面正常但 5 秒误报 [WATCHDOG] 强退 | g_last_frame_ms 刷新语句被误写进 frame.empty() 失败分支 | yolo_main.cpp captureLoop：刷新移到取帧成功路径 |
| 3 | 启动前 2 秒拖影/底部糊块 | 中途加入缺参考帧（等 IDR 约 1.2s）+ UDP 接收缓冲溢出丢包（IDR 约 130 包突发） | 中转端 onFrame 加「等 IDR 再送显」门控；udpsrc buffer-size=2097152；内核 rmem 8MB；output-corrupt=false；中转端队列溢出改为整体丢弃+重新等 IDR |
| 4 | 与上位机 DDS 互相发现不了 | 多网卡（eth0/eth1）下 CycloneDDS 0.7 绑定接口 Segfault、选错网卡；组播被网络环境影响 | 双端 Fast DDS + 静态 locator 绑 192.168.0.6 与 192.168.0.101；ros2 topic list 前 ros2 daemon stop |

严禁回退的改动：#1 的 set 保护、#2 的刷新位置、#3 的 IDR 门控与
buffer-size、#4 的 fastrtps_profile.xml。回退任一项都会复现对应故障。

## 7. 已知良性告警（见到可忽略）

- Failed to load plugin libgstopengl/libcluttergst3/libgstgtk ...
  cannot allocate memory in static TLS block：插件扫描期加载失败，
  链路不用这些插件
- AT-SPI: Error retrieving accessibility bus address：GTK 无障碍服务缺失
- Cannot query video position ... duration=-1：UDP 直播流无时长，
  OpenCV 转发的 GStreamer warning
- getProperty ... unhandled property: 6：查询 FOURCC 不被 GStreamer 后端支持

判真原则：只要「已等到首个 IDR」和 [DET] 持续输出，上述 WARN 均可无视；
出现 ERROR 级、[WATCHDOG]、「读取到空图像」才是真故障。

## 8. 验证与排障速查

    # 帧率 / 格式
    ros2 topic hz /drone1/camera_rgb/image_raw/compressed           # 约 25
    ros2 topic echo /drone1/camera_rgb/image_raw/compressed | head -20
    # 期望 format: h264，data 开头 0 0 0 1 ...

    # 丢包（跑几分钟前后各看一次，增量应为 0 或个位数）
    nstat -az | grep UdpRcvbufErrors

    # DDS 发现不了话题
    ros2 daemon stop && ros2 topic list -t
    echo $FASTRTPS_DEFAULT_PROFILES_FILE   # 确认非空
    ping 192.168.0.101

故障速判：

1. 画面拖影/糊块 -> 查 UdpRcvbufErrors 增量；必要时 buffer-size 提到
   4194304、rmem_max 提到 16777216
2. 5 秒 WATCHDOG 强退 -> 先看中转端统计行是否停（上游断流）vs
   ros2 topic hz 是否还在（DDS 断流）
3. 画面黑但无 WATCHDOG -> 等 IDR 窗口（约 1.2s）内属正常，超 3s 查中转端日志

## 9. 性能现状

| 指标 | 实测值 |
|---|---|
| 采集帧率 | 25 fps（源头 25fps，非丢帧） |
| 推理/显示刷新率 | 约 13.8 fps（单线程推理瓶颈，[DET] FPS= 即显示刷新率） |
| 输入队列丢帧 | 有（input_queue 容量 10、tryPopLatest 取最新，设计内降级保低延迟） |
| 检测结果 | 检测数量=0 属正常（测试视频内无目标物） |
| 串口 | /dev/ttyS9 115200，字符模式无目标不发数据；对端发 0x11 切字节流模式后每帧发 21 字节包（含空帧心跳），0x10 切回字符模式 |

## 10. 后续待办（不在本次备份范围）

- [ ] 雷达点云订阅 + 障碍物坐标计算 + 避障轨迹串口输出（方案已梳理，未实施）
- [ ] 可选：显示与推理解耦 / 多 NPU 核心并行提升显示帧率（当前不优化）
- [ ] 可选：中转端开机自启 systemd 服务
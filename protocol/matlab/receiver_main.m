function receiver_main()
%RECEIVER_MAIN 仿真机端 UDP 接收主循环
%   监听 0.0.0.0:8080，接收控制帧 -> 解析 -> 分发到电机控制。
%   电机控制接口尚未确定（方案第 5 点），当前为占位，仅打印指令。
%   需 MATLAB R2020b+（udpport）。按 Ctrl+C 停止。

u = udpport("datagram", "LocalHost", "0.0.0.0", "LocalPort", 8080);
fprintf('[receiver] 监听 UDP 0.0.0.0:8080 ...\n');
fprintf('[receiver] 按 Ctrl+C 停止\n');

last_tick = 0;
while true
    if u.NumDatagramsAvailable > 0
        data = read(u, u.NumDatagramsAvailable, "uint8");
        for k = 1:numel(data)
            r = parse_frame(data{k});
            if ~r.ok
                fprintf('[receiver] 丢弃帧：%s\n', r.err);
                continue;
            end
            dtick = mod(r.tick - last_tick, 2^32);   % uint32 差值，正确处理溢出回绕
            if dtick > 2^31
                fprintf('[receiver] 警告：时标异常跳变（差 %d tick）\n', dtick);
            end
            last_tick = r.tick;
            dispatch(r);
        end
    else
        pause(0.001);
    end
end
end

function dispatch(r)
%DISPATCH 按 cmd 分发指令（电机控制占位）
switch r.cmd
    case 1   % 速度指令
        v = r.payload;   % [vx vy vz vyaw]
        % TODO: 调用电机速度控制接口（方案第 5 点待定后填入）
        fprintf('[ctrl] 速度 vx=%.3f vy=%.3f vz=%.3f vyaw=%.3f (tick=%d)\n', ...
            v(1), v(2), v(3), v(4), r.tick);
    case 2   % 航迹点
        pts = r.payload;   % n×3 [x y z]
        % TODO: 调用航迹控制接口（可复用 trajectory 生成，或直接下发位置）
        fprintf('[ctrl] 航迹点 %d 个 (tick=%d)\n', size(pts, 1), r.tick);
    case 3   % 悬停
        % TODO: 调用电机悬停接口
        fprintf('[ctrl] 悬停 (tick=%d)\n', r.tick);
    case 4   % 模式切换
        if r.payload == 1
            fprintf('[ctrl] 模式切换：中断航迹，进入跟踪\n');
        else
            fprintf('[ctrl] 模式切换：恢复航迹\n');
        end
end
end

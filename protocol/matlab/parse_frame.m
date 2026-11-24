function result = parse_frame(bytes)
%PARSE_FRAME 解析一帧控制帧（协议 v1，小端，1 字节累加和取反）
%   输入 bytes: uint8 向量（一帧完整字节）
%   输出 result: 结构体
%       .ok       逻辑，是否解析成功（帧头/长度/校验均通过）
%       .cmd      double，命令类型（1=速度 2=航迹点 3=悬停 4=模式切换）
%       .tick     double，时标（每 10ms 递增 1）
%       .payload  double 数组，已解析的物理量（依 cmd 而定）
%       .err      字符串，错误信息（ok=false 时）

result = struct('ok', false, 'cmd', 0, 'tick', 0, 'payload', [], 'err', '');

if isempty(bytes) || numel(bytes) < 12
    result.err = '帧长不足 12 字节';
    return;
end

bytes = uint8(bytes(:));   % 统一为列向量

% 1) 帧头
if bytes(1) ~= hex2dec('A5') || bytes(2) ~= hex2dec('DA')
    result.err = '帧头错误';
    return;
end

% 2) 整帧长度（uint16 小端）
frame_len = double(typecast(bytes(3:4), 'uint16'));
if frame_len ~= numel(bytes)
    result.err = sprintf('长度字段=%d 与实收 %d 不符', frame_len, numel(bytes));
    return;
end

% 3) 校验（整帧累加 mod 256 必须为 0）
if mod(sum(double(bytes)), 256) ~= 0
    result.err = '校验失败';
    return;
end

% 4) 时标（uint32 小端）
result.tick = double(typecast(bytes(5:8), 'uint32'));

% 5) cmd（偏移 10）
result.cmd = double(bytes(11));

% 6) 载荷数据（偏移 11 … 倒数第二字节）
payload_bytes = bytes(12:end-1);

switch result.cmd
    case 1   % 速度指令：vx vy vz vyaw
        if numel(payload_bytes) ~= 16
            result.err = '速度指令载荷应为 16 字节';
            return;
        end
        result.payload = double(typecast(payload_bytes, 'single'));  % [vx vy vz vyaw]
    case 2   % 航迹点：n(1B) + n*(x y z)
        if isempty(payload_bytes)
            result.err = '航迹点载荷为空';
            return;
        end
        n = double(payload_bytes(1));
        if numel(payload_bytes) ~= 1 + n * 12
            result.err = sprintf('航迹点载荷长度错误：应 %d，实 %d', 1 + n * 12, numel(payload_bytes));
            return;
        end
        pts = typecast(payload_bytes(2:end), 'single');
        result.payload = reshape(pts, 3, n).';   % n×3 [x y z]
    case 3   % 悬停
        result.payload = [];
    case 4   % 模式切换：标志位(1B)
        if numel(payload_bytes) ~= 1
            result.err = '模式切换载荷应为 1 字节';
            return;
        end
        result.payload = double(payload_bytes(1));
    otherwise
        result.err = sprintf('未知 cmd=%d', result.cmd);
        return;
end

result.ok = true;
end

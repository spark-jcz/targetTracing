function frame = build_frame(tick, cmd, payload)
%BUILD_FRAME 打包一帧控制帧（协议 v1，数仿造帧用，镜像 C++ 端 frame_codec.h）
%   tick:    double，时标（uint32）
%   cmd:     double，命令类型（uint8）
%   payload: uint8 向量，载荷数据（可空，可缺省）
%   返回 frame: uint8 行向量（一帧完整字节）

if nargin < 3 || isempty(payload)
    payload = uint8([]);
end
payload = uint8(payload(:).');   % 行向量

total_len = 12 + numel(payload);
frame = zeros(1, total_len, 'uint8');

frame(1)   = hex2dec('A5');
frame(2)   = hex2dec('DA');
frame(3:4) = typecast(uint16(total_len), 'uint8');   % 小端
frame(5:8) = typecast(uint32(mod(tick, 2^32)), 'uint8');   % 小端；tick 从 0 递增，溢出回 0
frame(9)   = 0;
frame(10)  = 1;
frame(11)  = cmd;
if ~isempty(payload)
    frame(12 : 11 + numel(payload)) = payload;
end
frame(end) = uint8(mod(-sum(double(frame(1:end-1))), 256));   % 累加和取反
end

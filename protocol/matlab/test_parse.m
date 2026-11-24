% test_parse.m —— 数仿验证脚本
% 造各种帧，验证 parse_frame 解析正确（build_frame -> parse_frame 往返）
% 直接运行本脚本即可，全部通过会打印「全部数仿测试通过」。

%% 1) 速度指令帧
tick = 1000;
v = single([1.5, -0.3, 0.2, 0.05]);            % [vx vy vz vyaw]
frame = build_frame(tick, 1, typecast(v, 'uint8'));
r = parse_frame(frame);
assert(r.ok, r.err);
assert(r.cmd == 1);
assert(r.tick == tick);
assert(max(abs(r.payload - double(v))) < 1e-6);
fprintf('速度指令帧解析 OK\n');

%% 2) 悬停帧
frame = build_frame(1001, 3, []);
r = parse_frame(frame);
assert(r.ok, r.err);
assert(r.cmd == 3);
assert(r.tick == 1001);
assert(isempty(r.payload));
fprintf('悬停帧解析 OK\n');

%% 3) 模式切换帧
frame = build_frame(1002, 4, uint8(1));
r = parse_frame(frame);
assert(r.ok, r.err);
assert(r.cmd == 4);
assert(r.payload == 1);
fprintf('模式切换帧解析 OK\n');

%% 4) 航迹点帧（2 个点）
pts = single([0 0 0; 5 0 10]);                  % 2×3 [x y z]
n = size(pts, 1);
pts_vec = reshape(pts.', 1, []);                % [x1 y1 z1 x2 y2 z2]
payload = [uint8(n), typecast(pts_vec, 'uint8')];
frame = build_frame(1003, 2, payload);
r = parse_frame(frame);
assert(r.ok, r.err);
assert(r.cmd == 2);
assert(r.tick == 1003);
assert(size(r.payload, 1) == 2);
assert(max(abs(r.payload(:) - double(pts(:)))) < 1e-6);
fprintf('航迹点帧解析 OK\n');

%% 5) 错误帧：篡改校验
frame = build_frame(1004, 1, typecast(single([0 0 0 0]), 'uint8'));
frame(end) = bitxor(frame(end), uint8(1));      % 破坏校验字节（翻转最低位，避免 uint8 饱和）
r = parse_frame(frame);
assert(~r.ok && strcmp(r.err, '校验失败'));
fprintf('校验失败检测 OK\n');

%% 6) 错误帧：篡改长度字段
frame = build_frame(1005, 3, []);
frame(3) = frame(3) + 1;                        % 破坏长度字段
r = parse_frame(frame);
assert(~r.ok && contains(r.err, '长度字段'));
fprintf('长度错误检测 OK\n');

%% 7) 错误帧：错误帧头
frame = build_frame(1006, 3, []);
frame(1) = 0;                                   % 破坏帧头
r = parse_frame(frame);
assert(~r.ok && strcmp(r.err, '帧头错误'));
fprintf('帧头错误检测 OK\n');

%% 8) 与协议文档示例帧逐字节比对（速度指令 tick=1000）
expected = [hex2dec('A5') hex2dec('DA') hex2dec('1C') 0 hex2dec('E8') 3 0 0 0 1 1 ...
            hex2dec('00') 0 hex2dec('C0') hex2dec('3F') ...
            hex2dec('9A') hex2dec('99') hex2dec('99') hex2dec('BE') ...
            hex2dec('CD') hex2dec('CC') hex2dec('4C') hex2dec('3E') ...
            hex2dec('CD') hex2dec('CC') hex2dec('4C') hex2dec('3D')];
frame = build_frame(1000, 1, typecast(v, 'uint8'));
assert(isequal(frame(1:27), uint8(expected)));   % 前 27 字节逐字节比对，校验位已由 parse 验证
fprintf('与协议文档示例帧逐字节一致 OK\n');

%% 9) 时标溢出回 0
frame = build_frame(2^32, 3, []);   % tick = 2^32，应回绕为 0
r = parse_frame(frame);
assert(r.ok && r.tick == 0);
fprintf('时标溢出回 0 OK\n');

fprintf('\n全部数仿测试通过！\n');

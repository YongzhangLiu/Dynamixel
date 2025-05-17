clc;
clear;

% === Setup ===
lib_name = '';  % OS-dependent loading
if strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% === Config ===
PROTOCOL_VERSION = 2.0;
DXL_ID = 2;
DEVICENAME = 'COM1'; %--------- [INFO] Change this
BAUDRATE = 4500000;

ADDR = struct( ...
    'TORQUE_ENABLE',       64, ...
    'OPERATING_MODE',      11, ...
    'GOAL_PWM',           100, ...
    'PRESENT_POSITION',   132 ...
);

MODE = 16;  % PWM mode
THRESHOLD = 20;
ESC_CHARACTER = 'e';

% === Initialize Port ===
port_num = portHandler(DEVICENAME);
packetHandler();
if ~openPort(port_num), error('Failed to open port.'); end
if ~setBaudRate(port_num, BAUDRATE), error('Failed to set baudrate.'); end

% === Configure Motor ===
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR.OPERATING_MODE, MODE);
check_comm_result(port_num, PROTOCOL_VERSION);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR.TORQUE_ENABLE, 1);
check_comm_result(port_num, PROTOCOL_VERSION);

% === PID Gains ===
Kp = 2.0; Ki = 0.5; Kd = 0.1;

% === Main Loop ===
while true
    user_input = input('Enter position (or "e" to exit): ', 's');
    if strcmp(user_input, ESC_CHARACTER)
        break;
    end

    goal_pos = int32(str2double(user_input));
    moveToPositionPID(port_num, PROTOCOL_VERSION, DXL_ID, ADDR, goal_pos, Kp, Ki, Kd, THRESHOLD);
end

% === Cleanup ===
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR.TORQUE_ENABLE, 0);
check_comm_result(port_num, PROTOCOL_VERSION);
closePort(port_num);
unloadlibrary(lib_name);
clear;

% === PID Position Control Function ===
function moveToPositionPID(port_num, protocol, id, ADDR, goal_pos, Kp, Ki, Kd, threshold)
    t = tic;
    error_integral = 0;

    curr_pos = readPosition(port_num, protocol, id, ADDR.PRESENT_POSITION);
    error = goal_pos - curr_pos;
    error_prev = error;

    while abs(error) > threshold
        dt = toc(t); t = tic;
        if dt == 0, dt = eps; end

        error_integral = error_integral + error * dt;
        error_derivative = (error - error_prev) / dt;

        pwm = Kp * error + Ki * error_integral + Kd * error_derivative;
        pwm = map(pwm, -4096, 4096, -855, 855);

        write2ByteTxRx(port_num, protocol, id, ADDR.GOAL_PWM, typecast(int16(pwm), 'uint16'));
        check_comm_result(port_num, protocol);

        error_prev = error;
        curr_pos = readPosition(port_num, protocol, id, ADDR.PRESENT_POSITION);
        error = goal_pos - curr_pos;

        fprintf('[ID:%03d] Goal:%d  Pos:%d  PWM:%d\n', id, goal_pos, curr_pos, pwm);
    end

    % Stop motor
    write2ByteTxRx(port_num, protocol, id, ADDR.GOAL_PWM, 0);
    check_comm_result(port_num, protocol);
end
% === Position Read Function ===
function pos = readPosition(port_num, protocol, id, ADDR_PRESENT_POSITION)
    persistent last_valid_pos;
    raw = read4ByteTxRx(port_num, protocol, id, ADDR_PRESENT_POSITION);
    check = check_comm_result(port_num, protocol);
    if ~check
        if isempty(last_valid_pos)
            pos = 0;
        else
            pos = last_valid_pos;
        end
    else
        pos = typecast(uint32(raw), 'int32');
        last_valid_pos = pos;
    end
end
% === Check Comm Result Function ===
function result = check_comm_result(port_num, PROTOCOL_VERSION)
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    result = true;
    if dxl_comm_result ~= 0 % 0:=COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        result = false;
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        result = false;
    end
end
% === Map Clamp Function ===
function result = map(x, in_min, in_max, out_min, out_max)
    % improved map() function, when input exceeds specified range output is kept within limit
    if x < in_min 
        result = out_min;
        return;
    end
    if x > in_max
        result = out_max;
        return;
    end
    result =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return;
end

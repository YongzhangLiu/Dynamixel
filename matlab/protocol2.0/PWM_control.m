clc;
clear all;

% Includes
lib_name = '';
if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end
% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end


% General config
PROTOCOL_VERSION            = 2.0;          
DXL_ID                      = 2; % -----------------------------------[CHANGED]
DEVICENAME                  = 'COM3';  % -----------------------------[CHANGED]
% DEVICENAME                  = 'COM5';  % -----------------------------[CHANGED]
BAUDRATE                    = 4500000; % 4.5Mbps, 7 baudrate ---------[CHECK]

% Common Control Table Address and Data 
ADDR_TORQUE_ENABLE          = 64;
ADDR_GOAL_POSITION          = 116;
ADDR_GOAL_PWM               = 100;
ADDR_PRESENT_POSITION       = 132;

ADDR_OPERATING_MODE         = 11;          
OPERATING_MODE              = 16;            % value for operating mode for PWM control                                
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initializations
    % Initialize PortHandler Structs
    % Set the port path
    % Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);
    % Initialize PacketHandler Structs
packetHandler();
    % Variables initialization
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_pwm = 0;                           % Goal pwm (-855-855) ------[PWM]

dxl_error = 0;                              % Dynamixel error
dxl_error_prev = 0;
dxl_present_position = 0;                   % Present position
dxl_goal_position = 0;                      % goal position
    % PID
Kp = 2.0;
Ki = 0.5;
Kd = 0.1;
    % Timer
start_time = 0;
t = 0;

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set operating mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE);
if check_comm_result(port_num, PROTOCOL_VERSION) == true
    fprintf('Operating Mode successfully set \n');
end

% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
if check_comm_result(port_num, PROTOCOL_VERSION) == true
    fprintf('Dynamixel has been successfully connected \n');
end


while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Write goal position
    % dxl_goal_pwm = input('Input PWM Value\n');
    dxl_goal_position = int32(input('Input Position Value\n'));
    
    % Timer
    start_time = tic;
    t = tic;

    % Control Loop to realize position input
    while 1
        % PID Controller
        
        % Update previous error
        dxl_error_prev = dxl_error;

        dxl_goal_pwm = Kp * dxl_error + Ki * dxl_error * toc(start_time) + Kd * (dxl_error - dxl_error_prev) / toc(t);
        dxl_goal_pwm = map(dxl_goal_pwm, -4096, 4096, -855, 855); % Error 1 rev maxes out PWM output
        % Write PWM value
        write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_PWM, typecast(int16(dxl_goal_pwm), 'uint16'));
        check_comm_result(port_num, PROTOCOL_VERSION);
        t = tic;
       
        % Read present position
        % prev position to fix jerk issue
        dxl_present_position_prev = dxl_present_position;
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        dxl_present_position = typecast(uint32(dxl_present_position), 'int32');
        if ~check_comm_result(port_num, PROTOCOL_VERSION)
             dxl_present_position = dxl_present_position_prev;
        end
        
        % Error
        dxl_error = dxl_goal_position - dxl_present_position;

        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d, PresPWM: %03d\n', DXL_ID, dxl_goal_position, typecast(uint32(dxl_present_position), 'int32'), dxl_goal_pwm);

        if ~(abs(dxl_error) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end

    % Write PWM 0
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_PWM, 0);
    check_comm_result(port_num, PROTOCOL_VERSION);
end


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
check_comm_result(port_num, PROTOCOL_VERSION);

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;

% Functions
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


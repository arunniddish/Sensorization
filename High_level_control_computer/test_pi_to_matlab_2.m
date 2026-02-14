clear all;
clc;

tcp_1 = tcpserver('0.0.0.0', 5000);
tcp_2 = tcpserver('0.0.0.0', 5001);

% tcp_1.Timeout = 4;
% tcp_2.Timeout = 4;

disp('Waiting for connection...');

while ~tcp_1.Connected && ~tcp_2.Connected
    disp('Still trying to connect!!!');
    pause(0.5);
end

disp('### Now Connected ###');
disp(tcp_1.ClientAddress);
disp(tcp_2.ClientAddress);

tcp_1.flush;
tcp_2.flush;

% Read calibration status
ii = 0;
while true

    try
        if tcp_1.NumBytesAvailable >= 16
            raw_bytes = read(tcp_1, 16, "uint8");
            floats = typecast(uint8(raw_bytes), 'single');
            disp(floats);
        else
            ii = ii + 1;
            pause(0.05);
        end

        if ii == 100
            break;
        end

    catch
        break;
    end

end

clear floats;
tcp_1.flush;
pause(5);

% Read initial values
ii = 0;
jj = 0;
while true

    try
        if tcp_1.NumBytesAvailable >= 40
            raw_bytes = read(tcp_1, 40, "uint8");
            floats = typecast(uint8(raw_bytes), 'single');
            jj = jj+1;
            init_val(jj,:) = floats;
            disp(floats);
        else
            ii = ii + 1;
            pause(0.05);
        end
        if ii == 100
            break;
        end

    catch ME
        break;
    end

end

clear floats;
tcp_1.flush;
pause(3);

% Send BLE command to robot

% All data acquisition
flag = 0;

while true

    if tcp_1.NumBytesAvailable >= 10 && flag == 0
        raw_bytes = read(tcp_1, 56, "uint8");
        finish_check = typecast(uint8(raw_bytes), 'single');
        flag = 1;
    end

    if tcp_1.NumBytesAvailable >= 56 && strcmp(finish_check,"completed")
        raw_bytes = read(tcp_1, 56, "uint8");
        floats = typecast(uint8(raw_bytes), 'single');
        data(jj,:) = floats; % Store acquired data
        disp(floats);
        jj = jj + 1;
    else
        ii = ii + 1;
        pause(0.05);
    end
    if ii == 100
        break;
    end
end



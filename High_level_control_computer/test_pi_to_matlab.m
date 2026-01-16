clear all;
clc;

tcp_1 = tcpserver('0.0.0.0', 5000);
tcp_2 = tcpserver('0.0.0.0', 5001);
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

% Create figure for displaying image
figure('Name', 'Camera Feed');
img_handle = [];

ii = 1;
jj = 1;



% BLE setup for MATLAB to Seeeduino

device_name = "Seeeduino Sense Test";

list_of_devices = blelist;

check_device = strcmp(device_name,list_of_devices.Name);

while ~any(check_device)
    disp([device_name,"BLE device not found!!!"]);
end

disp([device_name,"BLE device found"]);


% Connect to BLE device
Msoro_BLE = ble(device_name);

while ~ Msoro_BLE.Connected
    disp('Still Connecting...')
end

disp('Connected Successfully');

Msoro_BLE_char = characteristic(Msoro_BLE, "C2FC88C4-F244-5A80-21F1-000224E80658","C2FC88C4-F244-5A80-21F1-000224E80658");  % c = characteristic(b,serviceName,characteristicName)





while true

    while ii == 1
        if tcp_1.NumBytesAvailable >= 16
            raw_bytes = read(tcp_1, 16, "uint8");
            floats = typecast(uint8(raw_bytes), 'single');
            flex_volt = floats(1);
            euler_angle = floats(2:4);
            disp([flex_volt, euler_angle]);
            ii = 0;
        end
    end

    while jj == 1
        if tcp_2.NumBytesAvailable >= 4
            size_bytes = read(tcp_2, 4, "uint8");
            img_size = typecast(uint8(size_bytes), 'uint32');
            % disp(['Incoming image size: ' num2str(img_size) ' bytes']);

            while tcp_2.NumBytesAvailable < img_size
                pause(0.01);
            end

            img_bytes = read(tcp_2, img_size, "uint8");
            % img = imdecode(uint8(img_bytes), 'jpg');
            
            % Saving to a temperoary file
            temp_file = 'temp_image.jpg';
            fid = fopen(temp_file, 'w');
            fwrite(fid, img_bytes, 'uint8');
            fclose(fid);

            jj = 0;
        end
    end
    
    img = imread(temp_file);

    img = insertText(img, [10, 10], sprintf('Flex Voltage: %.3f V', flex_volt), ...
                     'FontSize', 18, 'BoxColor', 'black', 'BoxOpacity', 0.7, 'TextColor', 'yellow');
    
    img = insertText(img, [10, 40], sprintf('Roll: %.2f°', euler_angle(1)), ...
                     'FontSize', 18, 'BoxColor', 'black', 'BoxOpacity', 0.7, 'TextColor', 'cyan');
    
    img = insertText(img, [10, 70], sprintf('Pitch: %.2f°', euler_angle(2)), ...
                     'FontSize', 18, 'BoxColor', 'black', 'BoxOpacity', 0.7, 'TextColor', 'cyan');
    
    img = insertText(img, [10, 100], sprintf('Yaw: %.2f°', euler_angle(3)), ...
                     'FontSize', 18, 'BoxColor', 'black', 'BoxOpacity', 0.7, 'TextColor', 'cyan');
    
    if isempty(img_handle)
        img_handle = imshow(img);
    else
        set(img_handle, 'CData', img);
    end
    drawnow;

    ii = 1;
    jj = 1;

end


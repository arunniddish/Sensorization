clear all;
clc;

all_data = load('flex_imu_list_26.csv');
pause(3);
xx = 1:size(all_data,1);
xx = xx';

% Normalize starting with 1 for all flex data
flex_data = max(all_data(:,1:4), 1);

% Four flex sensors
flex_1 = flex_data(:,1);
flex_2 = flex_data(:,2);
flex_3 = flex_data(:,3);
flex_4 = flex_data(:,4);

figure;
subplot(2,2,1)
plot(xx, flex_1(:,1), 'DisplayName','Signal')

subplot(2,2,2)
plot(xx, flex_2(:,1), 'DisplayName','Signal')

subplot(2,2,3)
plot(xx, flex_3(:,1), 'DisplayName','Signal')

subplot(2,2,4)
plot(xx, flex_4(:,1), 'DisplayName','Signal')

% Clear spikes and noises in peaks
med_filt_width = 3;
clean_data_1 = medfilt1(flex_1, med_filt_width);
clean_data_2 = medfilt1(flex_2, med_filt_width);
clean_data_3 = medfilt1(flex_3, med_filt_width);
clean_data_4 = medfilt1(flex_4, med_filt_width);

% Secondary filter to remove peaks
mode_1 = mode(flex_1(flex_1>1));
mode_2 = mode(flex_2(flex_2>1));
mode_3 = mode(flex_3(flex_3>1));
mode_4 = mode(flex_4(flex_4>1));

clean_data_1_new = min(clean_data_1, mode_1);
clean_data_2_new = min(clean_data_2, mode_2);
clean_data_3_new = min(clean_data_3, mode_3);
clean_data_4_new = min(clean_data_4, mode_4);

range_flex_1 = max(clean_data_1_new) - min(clean_data_1_new);
range_flex_2 = max(clean_data_2_new) - min(clean_data_2_new);
range_flex_3 = max(clean_data_3_new) - min(clean_data_3_new);
range_flex_4 = max(clean_data_4_new) - min(clean_data_4_new);

figure;
subplot(2,2,1)
plot(xx, clean_data_1_new(:,1), 'DisplayName','Signal')

subplot(2,2,2)
plot(xx, clean_data_2_new(:,1), 'DisplayName','Signal')

subplot(2,2,3)
plot(xx, clean_data_3_new(:,1), 'DisplayName','Signal')

subplot(2,2,4)
plot(xx, clean_data_4_new(:,1), 'DisplayName','Signal')

% Adding noise to find peaks
r = (0.0001 + (0.0002 - 0.0001) * rand(1, size(all_data,1)))';

clean_data_1_new = clean_data_1_new + r;
clean_data_2_new = clean_data_2_new + r;
clean_data_3_new = clean_data_3_new + r;
clean_data_4_new = clean_data_4_new + r;

figure;
subplot(2,2,1)
plot(xx, clean_data_1_new(:,1), 'DisplayName','Signal')

subplot(2,2,2)
plot(xx, clean_data_2_new(:,1), 'DisplayName','Signal')

subplot(2,2,3)
plot(xx, clean_data_3_new(:,1), 'DisplayName','Signal')

subplot(2,2,4)
plot(xx, clean_data_4_new(:,1), 'DisplayName','Signal')

% Peaks

threshold_1 = max(clean_data_1_new) - (1/5)*range_flex_1;
threshold_2 = max(clean_data_2_new) - (1/5)*range_flex_2;
threshold_3 = max(clean_data_3_new) - (1/5)*range_flex_3;
threshold_4 = max(clean_data_4_new) - (1/5)*range_flex_4;

[pks_1,plocs_1] = findpeaks(clean_data_1_new,"MinPeakHeight",threshold_1);
[pks_2,plocs_2] = findpeaks(clean_data_2_new,"MinPeakHeight",threshold_2);
[pks_3,plocs_3] = findpeaks(clean_data_3_new,"MinPeakHeight",threshold_3);
[pks_4,plocs_4] = findpeaks(clean_data_4_new,"MinPeakHeight",threshold_4);

figure;
subplot(2,2,1)
plot(xx, clean_data_1_new(:,1), 'DisplayName','Signal')
hold on
plot(xx(plocs_1), pks_1, '^r', 'DisplayName','Peaks')

subplot(2,2,2)
plot(xx, clean_data_2_new(:,1), 'DisplayName','Signal')
hold on
plot(xx(plocs_2), pks_2, '^r', 'DisplayName','Peaks')

subplot(2,2,3)
plot(xx, clean_data_3_new(:,1), 'DisplayName','Signal')
hold on
plot(xx(plocs_3), pks_3, '^r', 'DisplayName','Peaks')

subplot(2,2,4)
plot(xx, clean_data_4_new(:,1), 'DisplayName','Signal')
hold on
plot(xx(plocs_4), pks_4, '^r', 'DisplayName','Peaks')

% Robot state identification
limb_state_1 = zeros(size(flex_1,1),1);
limb_state_1(plocs_1) = 1;

limb_state_2 = zeros(size(flex_2,1),1);
limb_state_2(plocs_2) = 1;

limb_state_3 = zeros(size(flex_3,1),1);
limb_state_3(plocs_3) = 1;

limb_state_4 = zeros(size(flex_4,1),1);
limb_state_4(plocs_4) = 1;

window_size = 5; 
kernel = ones(window_size, 1);

smeared_1 = conv(double(limb_state_1), kernel, 'same') > 0;
smeared_2 = conv(double(limb_state_2), kernel, 'same') > 0;
smeared_3 = conv(double(limb_state_3), kernel, 'same') > 0;
smeared_4 = conv(double(limb_state_4), kernel, 'same') > 0;

overlap_region = smeared_2 & smeared_3 & smeared_1;

idx_logical_2 = find(overlap_region == 1);

d = diff(idx_logical_2);

switch_indices = find(d>10);
start_idx = 1;

for ii = 1:size(switch_indices,1)+1

    if ii ~= size(switch_indices,1)+1
        gaits_switch{ii} = idx_logical_2(start_idx:switch_indices(ii));
        mean_idx_val(ii) = round(mean(idx_logical_2(start_idx:switch_indices(ii))));
        start_idx = switch_indices(ii) + 1;
    else
        gaits_switch{ii} = idx_logical_2(start_idx:end);
        mean_idx_val(ii) = round(mean(idx_logical_2(start_idx:end)));
    end

end

% Find all pose data

for ii = 1:size(gaits_switch,2)
    imu_data{ii} = all_data(gaits_switch{1,ii}(1):gaits_switch{1,ii}(end),5:14);
end

% Calculate all average 

% [1] Average change in theta's

for ii = 1:size(imu_data,2)
    diff_theta{ii} = diff(imu_data{1,ii}(:,4:6));
    avg_diff_theta{ii} = mean(diff_theta{ii})*size(imu_data{1,ii},1);
end

% [2] Average linear acceleration

for ii = 1:size(imu_data,2)
    avg_lin_accel{ii} = mean(imu_data{1,ii}(:,1:3));
end

% Pose data
t = 0.450;
for ii = 1:size(gaits_switch,2)
    body_disp(ii,:) = 0.5*[avg_lin_accel{1,ii}(1),...
        avg_lin_accel{1,ii}(2)]*(t)^2;
end



%%
other_data = aa(203:219,5:14);

% Z - gives the orientation of the robot. (Yaw)
% Y - gives the (Pitch)
% X - gives the (Roll)



% Initial reading

init_accel_x = 
init_accel_y =
init_accel_z = 

init_theta_z = 
init_theta_y = 
init_theta_x = 

gait_prim_idx = 0;

for ii = 1:size(gaits_switch,2)
    
    v_init = [0,0,0];
        
    for jj = 1:size(gaits_switch{1,ii},1)

        if jj == 1
            delta_t = 0.450/size(gaits_switch{1,ii},1);
        end
        
        Rot_z = init_theta_z - aa(gaits_switch{1,ii}(jj,8)); 
        Rot_y = init_theta_y - aa(gaits_switch{1,ii}(jj,9));
        Rot_x = init_theta_x - aa(gaits_switch{1,ii}(jj,10));

        R_z = [cos(Rot_z) -sin(Rot_z) 0; sin(Rot_z) cos(Rot_z) 0; 0 0 1];
        R_y = [cos(Rot_y) 0 sin(Rot_y); 0 1 0; -sin(Rot_y) 0 cos(Rot_y)];
        R_x = [1 0 0; 0 cos(Rot_x) -sin(Rot_x); 0 sin(Rot_x) cos(Rot_x)];

        R_eq = R_z*R_y*R_x;
        a_new = R_eq*[aa(gaits_switch{1,ii}(jj,5));aa(gaits_switch{1,ii}(jj,6));aa(gaits_switch{1,ii}(jj,7))];
        a_new = a_new - [init_accel_x ; init_accel_y ; init_accel_z];

        v_init(1) = v_init(1) + a_new(1)*delta_t;
        v_init(2) = v_init(2) + a_new(2)*delta_t;
        v_init(3) = v_init(3) + a_new(3)*delta_t;

        delta_x = v_init(1)*delta_t + 0.5*a_new(1)*(delta_t)^2; 
        delta_y = v_init(2)*delta_t + 0.5*a_new(2)*(delta_t)^2;
        delta_z = v_init(3)*delta_t + 0.5*a_new(3)*(delta_t)^2;

        all_delta_x(gait_prim_idx + 1) = delta_x;
        all_delta_y(gait_prim_idx + 1) = delta_y;
        all_delta_z(gait_prim_idx + 1) = delta_z;

    end
end
            




        
           
        



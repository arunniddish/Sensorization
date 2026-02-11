aa = load('flex_volt_2.csv');
xx = 1:size(aa,1);
xx = xx';

% figure;
% plot(xx(:,1),aa(:,1));
% hold on
% plot(xx(:,1),aa(:,2));
% plot(xx(:,1),aa(:,3));
% plot(xx(:,1),aa(:,4));


[pks,plocs] = findpeaks(aa(:,2), 'MinPeakProminence',0.75);

[vys,vlocs] = findpeaks(aa(:,2), 'MinPeakProminence',0.75);

figure
plot(xx,aa(:,2), 'DisplayName','Signal')
hold on
plot(xx(plocs), pks, '^r', 'DisplayName','Peaks')
plot(xx(vlocs), -vys, 'vr', 'DisplayName','Valleys')
hold off
legend('Location','best')

figure
plot(xx(:,1),aa(:,2));
hold on
plot(xx(:,1),aa(:,2)+r);

r = (0.001 + (0.002 - 0.001) * rand(1, size(aa,1)))';

kk = aa(:,2)+r;

[pks,plocs] = findpeaks(kk);

[max_val, idx_max] = max(pks);
[min_val, idx_min] = min(pks);

range_val = max_val - min_val;

threshold_val = max_val - (1/8)*range_val;

yline(threshold_val);

[pks_mod,plocs_mod] = findpeaks(kk,"MinPeakHeight",threshold_val);

figure;
plot(xx(plocs_mod), pks_mod, '^r', 'DisplayName','Peaks')
hold on
plot(xx(:,1),kk);

xline(plocs_mod)

gaits_analyze = zeros(size(kk,1),1);
gaits_analyze(plocs_mod) = 1;




kk_2 = aa(:,2)+ (0.001 + (0.002 - 0.001) * rand(1, size(aa,1)))';
[pks_2,plocs_2] = findpeaks(kk_2);

[max_val_2, idx_max_2] = max(pks_2);
[min_val_2, idx_min_2] = min(pks_2);

range_val_2 = max_val_2 - min_val_2;

threshold_val_2 = max_val_2 - (1/8)*range_val_2;

[pks_mod_2, plocs_mod_2] = findpeaks(kk_2,"MinPeakHeight",threshold_val_2);

gaits_analyze_2 = zeros(size(kk_2,1),1);
gaits_analyze_2(plocs_mod_2) = 1;

peak_data = [gaits_analyze,gaits_analyze_2];


% Creating an intersection mask - Ideal situation

intersection_mask = peak_data(:,1) & peak_data(:,2);

idx_logical = find(intersection_mask == 1);

% Convoluted solution
sigA = peak_data(:,1);
sigB = peak_data(:,2);

window_size = 5; 
kernel = ones(window_size, 1);

smeared_A = conv(double(sigA), kernel, 'same') > 0;
smeared_B = conv(double(sigB), kernel, 'same') > 0;

overlap_region = smeared_A & smeared_B;

idx_logical_2 = find(overlap_region == 1);

% Finding clusters %

% for ii = 1:size(idx_logical_2,1) - 1
%     idx_step = idx_logical_2(ii) -  idx_logical_2(ii+1);

d = diff(idx_logical_2);

switch_indices = find(d>15);
start_idx = 1;
% gaits_switch = zeros(200, size(switch_indices,1)+1);

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





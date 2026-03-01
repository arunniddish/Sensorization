clear all;
clc;
%% Load and smooth data to square wave
aa = load('Euler_1_new_2.csv');  % or paste directly as a vector
data = aa(:,3);


% --- Parameters (tune these to your signal) ---
% threshold   = 1.05;   % midpoint between low (~1.0) and high (~1.15)
threshold = 1.025;
low_val     = mean(data(data < threshold));   % auto-detect LOW level
high_val    = mean(data(data >= threshold));  % auto-detect HIGH level

%% Method 1: Hard Threshold (simplest)
square_hard = double(data >= threshold);
square_hard = square_hard * (high_val - low_val) + low_val;

%% Method 2: Hysteresis Thresholding (avoids flickering at edges)
upper_thresh = low_val + 0.75 * (high_val - low_val);
lower_thresh = low_val + 0.25 * (high_val - low_val);

square_hyst = zeros(size(data));
state = data(1) >= threshold;  % initial state
for i = 1:length(data)
    if state == 0 && data(i) >= upper_thresh
        state = 1;
    elseif state == 1 && data(i) <= lower_thresh
        state = 0;
    end
    square_hyst(i) = state;
end
square_hyst = square_hyst * (high_val - low_val) + low_val;

%% Method 3: Median filter + threshold (smooths noise first)
window = 5;  % adjust window size
smoothed     = movmedian(data, window);
square_med   = double(smoothed >= threshold) * (high_val - low_val) + low_val;

%% Plot all methods
t = 1:length(data);

figure('Position', [100 100 1200 700]);

subplot(4,1,1)
plot(t, data, 'b', 'LineWidth', 1);
title('Original Signal'); ylabel('Amplitude'); grid on;

subplot(4,1,2)
plot(t, square_hard, 'r', 'LineWidth', 1.5);
title(sprintf('Method 1: Hard Threshold (%.3f)', threshold));
ylabel('Amplitude'); grid on; ylim([0.97 1.20]);

subplot(4,1,3)
plot(t, square_hyst, 'g', 'LineWidth', 1.5);
title('Method 2: Hysteresis Thresholding');
ylabel('Amplitude'); grid on; ylim([0.97 1.20]);

subplot(4,1,4)
plot(t, square_med, 'm', 'LineWidth', 1.5);
title(sprintf('Method 3: Median Filter (window=%d) + Threshold', window));
ylabel('Amplitude'); xlabel('Sample Index'); grid on; ylim([0.97 1.20]);

%% Print detected levels
fprintf('Detected LOW  level: %.6f\n', low_val);
fprintf('Detected HIGH level: %.6f\n', high_val);
fprintf('Threshold used:      %.6f\n', threshold);
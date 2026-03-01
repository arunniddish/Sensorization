%% ========================================================
%  Universal Square Wave Smoother - Auto-detects all levels
%  Works for any 2-state signal regardless of amplitude
%% ========================================================

function square = signal_to_square(data, method)
% SIGNAL_TO_SQUARE  Convert any noisy 2-level signal to a clean square wave
%
%   square = signal_to_square(data)         % uses Otsu (default)
%   square = signal_to_square(data, 'kmeans')
%   square = signal_to_square(data, 'otsu')
%   square = signal_to_square(data, 'hysteresis')
%
%   Works universally - no hardcoded thresholds or level values.

    if nargin < 2, method = 'otsu'; end

    data = data(:);  % ensure column vector
    N    = length(data);

    %% --- Step 1: Pre-smooth with median filter to reduce outlier influence ---
    win = max(3, round(N * 0.005));   % 0.5% of signal length, min 3
    if mod(win,2)==0, win = win+1; end
    data_smooth = medfilt1(data, win);

    %% --- Step 2: Find optimal threshold (chosen method) ---
    switch lower(method)

        case 'otsu'
            threshold = otsu_threshold(data_smooth);

        case 'kmeans'
            [~, centers] = kmeans(data_smooth, 2, 'Replicates', 5);
            threshold    = mean(centers);

        case 'hysteresis'
            base_thresh  = otsu_threshold(data_smooth);
            spread       = std(data_smooth);
            upper        = base_thresh + 0.5 * spread;
            lower        = base_thresh - 0.5 * spread;
            threshold    = base_thresh;  % stored for display
            % Hysteresis state machine
            binary = zeros(N,1);
            state  = data_smooth(1) > base_thresh;
            for i = 1:N
                if state==0 && data_smooth(i) > upper, state=1; end
                if state==1 && data_smooth(i) < lower, state=0; end
                binary(i) = state;
            end
            % Map to actual levels and return early
            low_val  = mean(data_smooth(binary==0));
            high_val = mean(data_smooth(binary==1));
            square   = binary * (high_val - low_val) + low_val;
            fprintf('[Hysteresis]  LOW=%.6f | HIGH=%.6f | Threshold≈%.6f\n', ...
                    low_val, high_val, base_thresh);
            return

        otherwise
            error('Unknown method. Choose: otsu | kmeans | hysteresis');
    end

    %% --- Step 3: Classify each sample into LOW or HIGH ---
    binary = double(data_smooth >= threshold);

    %% --- Step 4: Auto-detect actual LOW and HIGH levels ---
    low_val  = median(data_smooth(binary == 0));
    high_val = median(data_smooth(binary == 1));

    %% --- Step 5: Map to clean square wave ---
    square = binary * (high_val - low_val) + low_val;

    fprintf('[%-12s]  LOW=%.6f | HIGH=%.6f | Threshold=%.6f\n', ...
            upper(method), low_val, high_val, threshold);
end


%% ========================================================
%  Otsu's Method - Finds optimal threshold by maximizing
%  between-class variance (no assumptions about values)
%% ========================================================
function threshold = otsu_threshold(data)
    nbins     = 256;
    [counts, edges] = histcounts(data, nbins, 'Normalization', 'probability');
    bin_centers     = (edges(1:end-1) + edges(2:end)) / 2;

    % Cumulative sums
    w0  = cumsum(counts);                    % weight class 0
    w1  = 1 - w0;                           % weight class 1
    mu0 = cumsum(counts .* bin_centers) ./ (w0 + eps);
    mu1 = (sum(counts .* bin_centers) - cumsum(counts .* bin_centers)) ./ (w1 + eps);

    % Between-class variance
    sigma_b2 = w0 .* w1 .* (mu0 - mu1).^2;

    [~, idx]  = max(sigma_b2);
    threshold = bin_centers(idx);
end


%% ========================================================
%  MAIN SCRIPT - Load data and compare all methods
%% ========================================================

% --- Load your data (auto-handles file or variable) ---
% Option A: from file
% data = load('your_file.txt');

% Option B: paste directly (replace with your variable)
data = aa(:,3);  % <-- change this to your variable name

data = data(:);  % flatten to column vector

%% --- Run all 3 methods ---
sq_otsu  = signal_to_square(data, 'otsu');
sq_km    = signal_to_square(data, 'kmeans');
sq_hyst  = signal_to_square(data, 'hysteresis');

%% --- Detect transitions for annotation ---
trans = find(diff(sq_otsu) ~= 0);
fprintf('\nDetected %d transitions (Otsu method)\n', length(trans));
fprintf('Estimated period: ~%.0f samples\n', mean(diff(trans))*2);

%% --- Plot ---
t = 1:length(data);

figure('Color','w', 'Position',[50 50 1400 750]);

ax1 = subplot(4,1,1);
plot(t, data, 'Color',[0.3 0.5 0.9], 'LineWidth', 0.8);
title('Original Signal','FontSize',12,'FontWeight','bold');
ylabel('Value'); grid on; box off;

ax2 = subplot(4,1,2);
plot(t, sq_otsu, 'r', 'LineWidth', 2);
title('Otsu Threshold (Recommended)','FontSize',12,'FontWeight','bold');
ylabel('Value'); grid on; box off;

ax3 = subplot(4,1,3);
plot(t, sq_km, 'Color',[0.1 0.7 0.3], 'LineWidth', 2);
title('K-Means Clustering','FontSize',12,'FontWeight','bold');
ylabel('Value'); grid on; box off;

ax4 = subplot(4,1,4);
plot(t, sq_hyst, 'Color',[0.8 0.4 0.1], 'LineWidth', 2);
title('Hysteresis Thresholding','FontSize',12,'FontWeight','bold');
ylabel('Value'); xlabel('Sample Index'); grid on; box off;

linkaxes([ax1 ax2 ax3 ax4], 'x');  % sync zoom across all plots
sgtitle('Signal \rightarrow Square Wave (Fully Automatic)', 'FontSize', 14);

%% --- Export result ---
% writematrix(sq_otsu, 'square_wave_output.txt');
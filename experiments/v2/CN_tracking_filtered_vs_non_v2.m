%% HRI Filter Comparison V2 (Fair Start Alignment)
% Compares EKF+FIR filter vs non-filtered with fair start alignment
clear; clc; close all;

fileNonFilt = 'tracking_error_non_filtered_v2.csv';
fileFilt    = 'tracking_error_ekf_fir_v2.csv';

fprintf('Loading V2 experiment data (Fair Start Aligned)...\n');

%% Load Data
if exist(fileNonFilt, 'file')
    dataOld = readtable(fileNonFilt);
    ideal_Old = [dataOld.Ideal_X, dataOld.Ideal_Y, dataOld.Ideal_Z];
    real_Old  = [dataOld.Real_X,  dataOld.Real_Y,  dataOld.Real_Z];
    noisy_Old = [dataOld.Noisy_X, dataOld.Noisy_Y, dataOld.Noisy_Z];
    filt_Old  = [dataOld.Filt_X,  dataOld.Filt_Y,  dataOld.Filt_Z];
else
    error('Non-filtered CSV not found!');
end

if exist(fileFilt, 'file')
    dataNew = readtable(fileFilt);
    ideal_New = [dataNew.Ideal_X, dataNew.Ideal_Y, dataNew.Ideal_Z];
    real_New  = [dataNew.Real_X,  dataNew.Real_Y,  dataNew.Real_Z];
    noisy_New = [dataNew.Noisy_X, dataNew.Noisy_Y, dataNew.Noisy_Z];
    filt_New  = [dataNew.Filt_X,  dataNew.Filt_Y,  dataNew.Filt_Z];
else
    error('Filtered CSV not found!');
end

%% Spatial Mapping Error
fprintf('Calculating spatial errors...\n');
num_pts_old = size(real_Old, 1);
err_old = zeros(num_pts_old, 1);
for i = 1:num_pts_old
    dists = sqrt(sum((ideal_Old - real_Old(i,:)).^2, 2));
    err_old(i) = min(dists) * 1000;
end

num_pts_new = size(real_New, 1);
err_new = zeros(num_pts_new, 1);
for i = 1:num_pts_new
    dists = sqrt(sum((ideal_New - real_New(i,:)).^2, 2));
    err_new(i) = min(dists) * 1000;
end

%% Command Signal Error (filter's direct benefit)
err_cmd_old = vecnorm(filt_Old - ideal_Old, 2, 2) * 1000;
err_cmd_new = vecnorm(filt_New - ideal_New, 2, 2) * 1000;

%% Jitter
jitter_old = std(vecnorm(diff(real_Old), 2, 2)) * 1000;
jitter_new = std(vecnorm(diff(real_New), 2, 2)) * 1000;

%% Statistics
stats = struct();
stats.meanOld = mean(err_old);    stats.meanNew = mean(err_new);
stats.stdOld  = std(err_old);     stats.stdNew  = std(err_new);
stats.p95Old  = prctile(err_old, 95); stats.p95New = prctile(err_new, 95);
stats.maxOld  = max(err_old);     stats.maxNew  = max(err_new);
stats.cmdOld  = mean(err_cmd_old); stats.cmdNew = mean(err_cmd_new);

fprintf('\n=================================================================\n');
fprintf('     V2 HRI Filter Comparison Report (Fair Start Aligned)\n');
fprintf('=================================================================\n');
fprintf('%-30s | %-15s | %-15s | %-15s\n', 'Metric (mm)', 'Non-Filtered', 'EKF+FIR', 'Improvement');
fprintf('-----------------------------------------------------------------\n');
fprintf('%-30s | %-15.2f | %-15.2f | %.1f%%\n', 'Spatial Mean Error', stats.meanOld, stats.meanNew, (stats.meanOld-stats.meanNew)/stats.meanOld*100);
fprintf('%-30s | %-15.2f | %-15.2f | %.1f%%\n', 'Spatial STD', stats.stdOld, stats.stdNew, (stats.stdOld-stats.stdNew)/stats.stdOld*100);
fprintf('%-30s | %-15.2f | %-15.2f | %.1f%%\n', 'Spatial P95', stats.p95Old, stats.p95New, (stats.p95Old-stats.p95New)/stats.p95Old*100);
fprintf('%-30s | %-15.2f | %-15.2f | %.1f%%\n', 'Spatial Max', stats.maxOld, stats.maxNew, (stats.maxOld-stats.maxNew)/stats.maxOld*100);
fprintf('%-30s | %-15.2f | %-15.2f | %.1f%%\n', 'Command Mean Error', stats.cmdOld, stats.cmdNew, (stats.cmdOld-stats.cmdNew)/stats.cmdOld*100);
fprintf('%-30s | %-15.2f | %-15.2f | %.1f%%\n', 'Trajectory Jitter', jitter_old, jitter_new, (jitter_old-jitter_new)/jitter_old*100);
fprintf('=================================================================\n');

%% Plotting
figure('Color', 'w', 'Position', [50, 50, 1500, 500], 'Name', 'V2 Filter Comparison (Fair Start)');

% Subplot 1: Boxplot
subplot(1, 3, 1);
max_len = max(length(err_old), length(err_new));
b1 = [err_old; NaN(max_len-length(err_old),1)];
b2 = [err_new; NaN(max_len-length(err_new),1)];
boxplot([b1, b2], 'Labels', {'Non-Filtered', 'EKF+FIR'});
ylabel('Spatial Error (mm)');
title('V2: Spatial Mapping Error Distribution');
grid on;

% Subplot 2: Command signal error over time
subplot(1, 3, 2);
hold on; grid on;
scatter(1:num_pts_old, err_cmd_old, 10, [0.8 0.4 0.4], 'filled', 'DisplayName', 'Non-Filtered Cmd Error');
plot(1:num_pts_new, err_cmd_new, 'Color', [0.2 0.6 0.2], 'LineWidth', 2, 'DisplayName', 'EKF+FIR Cmd Error');
xlabel('Sample'); ylabel('Command Error (mm)');
title('V2: Command Signal Error Comparison');
legend('Location', 'northeast');

% Subplot 3: 3D trajectory
subplot(1, 3, 3);
hold on; grid on;
plot3(ideal_New(:,1), ideal_New(:,2), ideal_New(:,3), '--k', 'LineWidth', 1.5, 'DisplayName', 'Ideal');
plot3(real_Old(:,1), real_Old(:,2), real_Old(:,3), 'Color', [0.8 0.4 0.4 0.5], 'LineWidth', 1, 'DisplayName', 'Non-Filtered Real');
plot3(real_New(:,1), real_New(:,2), real_New(:,3), 'Color', [0.2 0.8 0.2], 'LineWidth', 2.5, 'DisplayName', 'EKF+FIR Real');
view(45, 30);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('V2: 3D Trajectory (Fair Start Aligned)');
legend('Location', 'best');
axis equal;

saveas(gcf, 'filter_comparison_v2.png');
fprintf('Plot saved as filter_comparison_v2.png\n');

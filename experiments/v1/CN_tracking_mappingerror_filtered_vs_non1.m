%% HRI Mapping and Robustness Evaluation: Anti-Mutation & Spatial Trajectory Comprehensive Analysis
clear; clc; close all;

% --- 1. Configuration ---
% Ensure these CSV files are generated using the latest Python scripts
fileNonFilt = 'tracking_error_non_filtered_v2.csv';     % Passthrough Version (With Jumps)
fileFilt    = 'tracking_error_kf_experiment2.0.csv'; % Filtering Version (With Jumps)

fprintf('Loading and processing the latest data...\n');

%% --- 2. Load Data ---
if exist(fileNonFilt, 'file')
    dataOld = readtable(fileNonFilt);
    ideal_Old = [dataOld.Ideal_X, dataOld.Ideal_Y, dataOld.Ideal_Z];
    real_Old  = [dataOld.Real_X,  dataOld.Real_Y,  dataOld.Real_Z];
else
    error('Error: Passthrough version CSV file not found!');
end

if exist(fileFilt, 'file')
    dataNew = readtable(fileFilt);
    ideal_New = [dataNew.Ideal_X, dataNew.Ideal_Y, dataNew.Ideal_Z];
    real_New  = [dataNew.Real_X,  dataNew.Real_Y,  dataNew.Real_Z];
else
    error('Error: Filtering version CSV file not found!');
end

%% --- 3. Core Algorithm: Pure Spatial Mapping Error ---
fprintf('Calculating spatial path errors for Passthrough Version...\n');
num_pts_old = size(real_Old, 1);
err_spatial_old = zeros(num_pts_old, 1);
for i = 1:num_pts_old
    dists = sqrt(sum((ideal_Old - real_Old(i,:)).^2, 2));
    err_spatial_old(i) = min(dists) * 1000; % Distance to nearest point, converted to mm
end

fprintf('Calculating spatial path errors for Filtering Version...\n');
num_pts_new = size(real_New, 1);
err_spatial_new = zeros(num_pts_new, 1);
for i = 1:num_pts_new
    dists = sqrt(sum((ideal_New - real_New(i,:)).^2, 2));
    err_spatial_new(i) = min(dists) * 1000; % Converted to mm
end

%% --- 4. Core Algorithm: Trajectory Jitter Rate ---
jitter_old = std(vecnorm(diff(real_Old), 2, 2)) * 1000; 
jitter_new = std(vecnorm(diff(real_New), 2, 2)) * 1000; 

%% --- 5. Statistics & Report Generation ---
stats = struct();

% --- Passthrough Version Stats ---
stats.meanOld = mean(err_spatial_old);
stats.stdOld  = std(err_spatial_old);    
stats.varOld  = var(err_spatial_old);    
stats.p95Old  = prctile(err_spatial_old, 95); 
stats.maxOld  = max(err_spatial_old);

% --- Filtering Version Stats ---
stats.meanNew = mean(err_spatial_new);
stats.stdNew  = std(err_spatial_new);    
stats.varNew  = var(err_spatial_new);    
stats.p95New  = prctile(err_spatial_new, 95);
stats.maxNew  = max(err_spatial_new);

% --- Improvement Rate Calculation ---
improve_mean = (stats.meanOld - stats.meanNew) / stats.meanOld * 100;
improve_std  = (stats.stdOld - stats.stdNew) / stats.stdOld * 100; 
improve_var  = (stats.varOld - stats.varNew) / stats.varOld * 100; 
improve_p95  = (stats.p95Old - stats.p95New) / stats.p95Old * 100;
improve_max  = (stats.maxOld - stats.maxNew) / stats.maxOld * 100;
improve_jitter = (jitter_old - jitter_new) / jitter_old * 100;

fprintf('\n=================================================================\n');
fprintf('        HRI Mapping System Robustness In-depth Statistical Report\n');
fprintf('=================================================================\n');
fprintf('%-30s | %-15s | %-15s | %-15s\n', 'Spatial Mapping Metrics (mm)', 'Passthrough', '3-Stage Filter', 'Improvement');
fprintf('-----------------------------------------------------------------\n');
fprintf('%-32s | %-15.2f | %-15.2f | %.1f%%\n', 'Mean Error (Mean)', stats.meanOld, stats.meanNew, improve_mean);
fprintf('%-32s | %-15.2f | %-15.2f | %.1f%% (*Dist. Convergence)\n', 'Error Std Dev (STD)', stats.stdOld, stats.stdNew, improve_std);
fprintf('%-32s | %-15.2f | %-15.2f | %.1f%% (*Outlier Elimination)\n', 'Error Variance (Variance)', stats.varOld, stats.varNew, improve_var);
fprintf('-----------------------------------------------------------------\n');
fprintf('%-32s | %-15.2f | %-15.2f | %.1f%%\n', '95th Percentile Error (P95)', stats.p95Old, stats.p95New, improve_p95);
fprintf('%-32s | %-15.2f | %-15.2f | %.1f%%\n', 'Max Jump Deviation (Max)', stats.maxOld, stats.maxNew, improve_max);
fprintf('%-32s | %-15.2f | %-15.2f | %.1f%%\n', 'Trajectory Jitter Rate (Jitter)', jitter_old, jitter_new, improve_jitter);
fprintf('=================================================================\n');

%% --- 6. Plotting: Visual Analysis ---
figure('Color', 'w', 'Position', [50, 50, 1500, 500], 'Name', 'Robustness and Mapping Performance Analysis');

% --- Subplot 1: Boxplot (Distribution of Anomalies) ---
subplot(1, 3, 1);
max_len = max(length(err_spatial_old), length(err_spatial_new));
box_data_old = [err_spatial_old; NaN(max_len - length(err_spatial_old), 1)];
box_data_new = [err_spatial_new; NaN(max_len - length(err_spatial_new), 1)];

boxplot([box_data_old, box_data_new], 'Labels', {'Passthrough (Exposed)', 'Filtered (Protected)'});
ylabel('Pure Spatial Deviation (mm)');
title('Spatial Mapping Error Distribution (Note Red + Outliers)');
grid on;

% --- Subplot 2: Spatial Error Spikes ---
subplot(1, 3, 2);
hold on; grid on;
scatter(1:num_pts_old, err_spatial_old, 15, [0.8 0.4 0.4], 'filled', 'DisplayName', 'Passthrough (Impacted by Jumps)');
plot(1:num_pts_new, err_spatial_new, 'Color', [0.2 0.6 0.2], 'LineWidth', 2.5, 'DisplayName', 'Filtered (Stable)');
xlabel('Data Sample Points (Chronological)'); ylabel('Spatial Deviation (mm)');
title('System Performance Under 6cm Jump Spikes');
legend('Location', 'northeast');

% --- Subplot 3: 3D Trajectory Map ---
subplot(1, 3, 3);
hold on; grid on;
plot3(ideal_New(:,1), ideal_New(:,2), ideal_New(:,3), '--k', 'LineWidth', 1.5, 'DisplayName', 'Ideal Ground Truth Arc');
plot3(real_Old(:,1), real_Old(:,2), real_Old(:,3), 'Color', [0.8 0.4 0.4 0.6], 'LineWidth', 1, 'DisplayName', 'Passthrough Trajectory (With Spikes)');
plot3(real_New(:,1), real_New(:,2), real_New(:,3), 'Color', [0.2 0.8 0.2], 'LineWidth', 2.5, 'DisplayName', 'Filtered Trajectory (Smooth)');
view(45, 30);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Spatial Trajectory Mapping Comparison');
legend('Location', 'best');
axis equal;
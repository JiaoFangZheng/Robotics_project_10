%% V3: Non-Filtered vs Kalman Filter (KF + FIR Kaiser + Threshold MA)
clear; clc; close all;

fileNonFilt = 'tracking_error_non_filtered_v3.csv';
fileFilt    = 'tracking_error_kf_v3.csv';

fprintf('Loading V3 experiment data (KF vs Non-Filtered)...\n');

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
    error('KF CSV not found!');
end

%% Spatial Mapping Error
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

%% Command Signal Error
err_cmd_old = vecnorm(filt_Old - ideal_Old, 2, 2) * 1000;
err_cmd_new = vecnorm(filt_New - ideal_New, 2, 2) * 1000;

%% Jitter
jitter_old = std(vecnorm(diff(real_Old), 2, 2)) * 1000;
jitter_new = std(vecnorm(diff(real_New), 2, 2)) * 1000;

%% Report
fprintf('\n=================================================================\n');
fprintf('     V3: Non-Filtered vs Kalman Filter\n');
fprintf('=================================================================\n');
fprintf('%-28s | %-13s | %-13s | %-10s\n', 'Metric (mm)', 'Non-Filtered', 'KF', 'Improve');
fprintf('-----------------------------------------------------------------\n');
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n', 'Spatial Mean', mean(err_old), mean(err_new), (mean(err_old)-mean(err_new))/mean(err_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n', 'Spatial STD', std(err_old), std(err_new), (std(err_old)-std(err_new))/std(err_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n', 'Spatial P95', prctile(err_old,95), prctile(err_new,95), (prctile(err_old,95)-prctile(err_new,95))/prctile(err_old,95)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n', 'Spatial Max', max(err_old), max(err_new), (max(err_old)-max(err_new))/max(err_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n', 'Cmd Mean Error', mean(err_cmd_old), mean(err_cmd_new), (mean(err_cmd_old)-mean(err_cmd_new))/mean(err_cmd_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n', 'Traj Jitter', jitter_old, jitter_new, (jitter_old-jitter_new)/jitter_old*100);
fprintf('=================================================================\n');

%% Plot
figure('Color','w','Position',[50,50,1500,500],'Name','V3 KF Comparison');

subplot(1,3,1);
max_len = max(length(err_old), length(err_new));
boxplot([[err_old; NaN(max_len-length(err_old),1)], [err_new; NaN(max_len-length(err_new),1)]], 'Labels',{'Non-Filtered','Kalman Filter'});
ylabel('Spatial Error (mm)'); title('V3: Spatial Mapping Error'); grid on;

subplot(1,3,2); hold on; grid on;
scatter(1:num_pts_old, err_cmd_old, 10, [0.8 0.4 0.4], 'filled', 'DisplayName', 'Non-Filtered');
plot(1:num_pts_new, err_cmd_new, 'Color', [0.2 0.2 0.8], 'LineWidth', 2, 'DisplayName', 'Kalman Filter');
xlabel('Sample'); ylabel('Cmd Error (mm)'); title('V3: Command Signal Error'); legend;

subplot(1,3,3); hold on; grid on;
plot3(ideal_New(:,1),ideal_New(:,2),ideal_New(:,3),'--k','LineWidth',1.5,'DisplayName','Ideal');
plot3(real_Old(:,1),real_Old(:,2),real_Old(:,3),'Color',[0.8 0.4 0.4 0.5],'LineWidth',1,'DisplayName','Non-Filtered');
plot3(real_New(:,1),real_New(:,2),real_New(:,3),'Color',[0.2 0.2 0.8],'LineWidth',2.5,'DisplayName','Kalman Filter');
view(45,30); xlabel('X'); ylabel('Y'); zlabel('Z'); title('V3: 3D Trajectory'); legend; axis equal;

saveas(gcf, 'filter_comparison_v3.png');
fprintf('Plot saved as filter_comparison_v3.png\n');

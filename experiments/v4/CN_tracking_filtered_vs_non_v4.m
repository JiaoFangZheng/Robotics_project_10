%% V4: Non-Filtered vs Pure Kalman Filter (No Innovation Gate)
clear; clc; close all;
fileNon = 'tracking_error_non_filtered_v4.csv';
fileKF   = 'tracking_error_kf_pure_v4.csv';
fprintf('Loading V4 (Pure KF, no gate)...\n');
dataN = readtable(fileNon); dataK = readtable(fileKF);
idN = [dataN.Ideal_X,dataN.Ideal_Y,dataN.Ideal_Z]; reN = [dataN.Real_X,dataN.Real_Y,dataN.Real_Z];
idK = [dataK.Ideal_X,dataK.Ideal_Y,dataK.Ideal_Z]; reK = [dataK.Real_X,dataK.Real_Y,dataK.Real_Z];
fN = [dataN.Filt_X,dataN.Filt_Y,dataN.Filt_Z]; fK = [dataK.Filt_X,dataK.Filt_Y,dataK.Filt_Z];

err_old=zeros(size(reN,1),1); for i=1:size(reN,1), dists=sqrt(sum((idN-reN(i,:)).^2,2)); err_old(i)=min(dists)*1000; end
err_new=zeros(size(reK,1),1); for i=1:size(reK,1), dists=sqrt(sum((idK-reK(i,:)).^2,2)); err_new(i)=min(dists)*1000; end
cmd_old=vecnorm(fN-idN,2,2)*1000; cmd_new=vecnorm(fK-idK,2,2)*1000;
j_old=std(vecnorm(diff(reN),2,2))*1000; j_new=std(vecnorm(diff(reK),2,2))*1000;

fprintf('\n=================================================================\n');
fprintf('     V4: Non-Filtered vs Pure Kalman Filter (No Innovation Gate)\n');
fprintf('=================================================================\n');
fprintf('%-28s | %-13s | %-13s | %-10s\n','Metric','Non-Filt','Pure KF','Improve');
fprintf('-----------------------------------------------------------------\n');
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Spatial Mean',mean(err_old),mean(err_new),(mean(err_old)-mean(err_new))/mean(err_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Spatial STD',std(err_old),std(err_new),(std(err_old)-std(err_new))/std(err_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Cmd Mean Error',mean(cmd_old),mean(cmd_new),(mean(cmd_old)-mean(cmd_new))/mean(cmd_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Cmd STD',std(cmd_old),std(cmd_new),(std(cmd_old)-std(cmd_new))/std(cmd_old)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Traj Jitter',j_old,j_new,(j_old-j_new)/j_old*100);
fprintf('=================================================================\n');

figure('Color','w','Position',[50,50,1500,500],'Name','V4 Pure KF');
subplot(1,3,1); ml=max(length(err_old),length(err_new));
boxplot([[err_old;NaN(ml-length(err_old),1)],[err_new;NaN(ml-length(err_new),1)]],'Labels',{'Non-Filt','Pure KF'});
ylabel('Spatial Error (mm)'); title('V4: Spatial Error (Pure KF, No Gate)'); grid on;
subplot(1,3,2); hold on; grid on;
plot(cmd_old,'Color',[0.8 0.4 0.4],'DisplayName','Non-Filt'); plot(cmd_new,'Color',[0.0 0.8 0.8],'LineWidth',2,'DisplayName','Pure KF');
xlabel('Sample'); ylabel('Cmd Error (mm)'); title('V4: Command Error'); legend;
subplot(1,3,3); hold on; grid on;
plot3(idK(:,1),idK(:,2),idK(:,3),'--k','DisplayName','Ideal');
plot3(reN(:,1),reN(:,2),reN(:,3),'Color',[0.8 0.4 0.4 0.5],'DisplayName','Non-Filt');
plot3(reK(:,1),reK(:,2),reK(:,3),'Color',[0.0 0.8 0.8],'LineWidth',2,'DisplayName','Pure KF');
view(45,30); xlabel('X'); ylabel('Y'); zlabel('Z'); title('V4: 3D Trajectory'); legend; axis equal;
saveas(gcf,'filter_comparison_v4.png'); fprintf('Saved filter_comparison_v4.png\n');

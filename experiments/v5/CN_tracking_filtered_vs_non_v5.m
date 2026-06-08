%% V5: Non-Filtered vs KF 2.0 (Distance Jump + FIR + Threshold MA)
clear; clc; close all;
dataN=readtable('tracking_error_non_filtered_v5.csv'); dataK=readtable('tracking_error_kf_v5.csv');
idN=[dataN.Ideal_X,dataN.Ideal_Y,dataN.Ideal_Z]; reN=[dataN.Real_X,dataN.Real_Y,dataN.Real_Z];
idK=[dataK.Ideal_X,dataK.Ideal_Y,dataK.Ideal_Z]; reK=[dataK.Real_X,dataK.Real_Y,dataK.Real_Z];
fN=[dataN.Filt_X,dataN.Filt_Y,dataN.Filt_Z]; fK=[dataK.Filt_X,dataK.Filt_Y,dataK.Filt_Z];
eo=zeros(size(reN,1),1); for i=1:size(reN,1), eo(i)=min(sqrt(sum((idN-reN(i,:)).^2,2)))*1000; end
en=zeros(size(reK,1),1); for i=1:size(reK,1), en(i)=min(sqrt(sum((idK-reK(i,:)).^2,2)))*1000; end
co=vecnorm(fN-idN,2,2)*1000; cn=vecnorm(fK-idK,2,2)*1000;
jo=std(vecnorm(diff(reN),2,2))*1000; jn=std(vecnorm(diff(reK),2,2))*1000;
fprintf('\n=================================================================\n     V5: Non-Filtered vs KF 2.0\n=================================================================\n');
fprintf('%-28s | %-13s | %-13s | %-10s\n','Metric','Non-Filt','KF 2.0','Improve');
fprintf('-----------------------------------------------------------------\n');
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Spatial Mean',mean(eo),mean(en),(mean(eo)-mean(en))/mean(eo)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Spatial STD',std(eo),std(en),(std(eo)-std(en))/std(eo)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Cmd Mean',mean(co),mean(cn),(mean(co)-mean(cn))/mean(co)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Cmd STD',std(co),std(cn),(std(co)-std(cn))/std(co)*100);
fprintf('%-28s | %-13.2f | %-13.2f | %.1f%%\n','Jitter',jo,jn,(jo-jn)/jo*100);
fprintf('=================================================================\n');
figure('Color','w','Position',[50,50,1500,500],'Name','V5 KF 2.0');
subplot(1,3,1); ml=max(length(eo),length(en));
boxplot([[eo;NaN(ml-length(eo),1)],[en;NaN(ml-length(en),1)]],'Labels',{'Non-Filt','KF 2.0'});
ylabel('Spatial Error (mm)'); title('V5: Spatial Error'); grid on;
subplot(1,3,2); hold on; grid on; plot(co,'r','DisplayName','Non-Filt'); plot(cn,'Color',[1 0.5 0],'LineWidth',2,'DisplayName','KF 2.0');
xlabel('Sample'); ylabel('Cmd Error (mm)'); title('V5: Command Error'); legend;
subplot(1,3,3); hold on; grid on; plot3(idK(:,1),idK(:,2),idK(:,3),'--k','DisplayName','Ideal');
plot3(reN(:,1),reN(:,2),reN(:,3),'Color',[0.8 0.4 0.4 0.5],'DisplayName','Non-Filt'); plot3(reK(:,1),reK(:,2),reK(:,3),'Color',[1 0.5 0],'LineWidth',2,'DisplayName','KF 2.0');
view(45,30); xlabel('X');ylabel('Y');zlabel('Z'); title('V5: 3D'); legend; axis equal;
saveas(gcf,'filter_comparison_v5.png'); fprintf('Saved.\n');

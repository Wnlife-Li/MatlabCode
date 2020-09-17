%%%%%%%%%%%%%%%%
%导入加速度数据并进行平滑滤波，加速度预处理数据
%%%%%%%%%%%%%%%%%%
newData = importdata('./T/T2.tsv', '\t', 2);
data=newData.data;

[m, n]=size(data);
% figure;
% subplot(3,1,1);
% plot(data(:,2));
% title('三轴加速度传感器获取的数据');
% xlabel('X轴采集数据');
% subplot(3,1,2);
% plot(data(:,3));
% xlabel('Y轴采集数据');
% ylabel('单位：m/s2');
% subplot(3,1,3);
% plot(data(:,4));
% xlabel('Z轴采集数据  单位:采样点');

%加速度幅值
asum=sqrt(data(:,2).^2+data(:,3).^2+data(:,4).^2);
% figure;
% plot(asum);
% title('三轴加速度的和加速度');
% xlabel('单位：采样点');
% ylabel('单位：m/s2');

g=1.02649;
%apsum减去重力以后的加速度
apsum=asum-g;

%平滑滤波
apsum_e=[];
 for i=1:m-10
     temp=sum(apsum(i:i+10,:))/11;  
     apsum_e = cat(1,apsum_e,temp);
 end
 
figure; 
% subplot(2,1,1);
plot(apsum);
title('三轴加速度的和去除重力以后的加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');
% subplot(2,1,2);
figure; 
plot(apsum_e);
title('平滑滤波后的三轴加速度的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%加速度预处理程序%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
%首先是卡尔曼滤波，然后是2次平滑滤波
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
 
figure; 
% subplot(2,1,1);
plot(apsum);
title('三轴加速度的和去除重力以后的加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');
% subplot(2,1,2);

[N,M]=size(apsum);

Q = 0.1;%系统噪声
R = 0.2;% 测量噪声

% 分配空间
xhat=zeros(N,1);       % x的后验估计
P=0.2;           % 后验方差估计  n*n
xhatminus=zeros(N,1);  % x的先验估计
Pminus=0;      % n*n
K=0;   % Kalman增益  n*m

% 估计的初始值都为默认的0，即P=[0 0;0 0],xhat=0
for k = 2:N         
    % 时间更新过程
    xhatminus(k) = xhat(k-1);
    Pminus= P+Q;
    
    % 测量更新过程
    K = Pminus*inv( Pminus+R );
    xhat(k) = xhatminus(k)+K*(apsum(k)-xhatminus(k));
    P = (1-K)*Pminus;
end
figure; 
plot(xhat);
title('卡尔曼滤波后的三轴加速度的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2'); 

% 平滑滤波1
apsum_e=[];
 for i=1:m-10
     temp=sum(xhat(i:i+10,:))/11;  
     apsum_e = cat(1,apsum_e,temp);
 end

figure; 
plot(apsum_e);
title('平滑滤波后的三轴加速度的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');

p=size(apsum_e);
% 平滑滤波2
apsum_e2=[];
 for i=1:p-4
     temp=sum(apsum_e(i:i+4,:))/5;  
     apsum_e2 = cat(1,apsum_e2,temp);
 end

figure; 
plot(apsum_e2);
title('平滑滤波后的三轴加速度的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');

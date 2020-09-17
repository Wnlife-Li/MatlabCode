clc;
clear all;

%惯性导航测试
newData = importdata('3.txt', '\t', 2);
data=newData.data;
ax=data(:,1);
ay=data(:,2);
az=data(:,3);
Angle=data(:,9);
%加速度幅值
a=sqrt(ax.^2+ay.^2+az.^2);
%均值
meanAx=mean(ax);
meanAy=mean(ay);
meanAz=mean(az);
meanA=mean(a);
meanAngle=mean(Angle);
%方差
axS=S(ax,meanAx);
ayS=S(ay,meanAy);
azS=S(az,meanAz);
aS=S(a,meanA);
AngleS=S(Angle,meanAngle);

figure;
subplot(4,1,1);
plot(ax);xlabel('采样点');ylabel('加速度/g');title('x轴加速度');
subplot(4,1,2);
plot(ay);xlabel('采样点');ylabel('加速度/g');title('y轴加速度');
subplot(4,1,3);
plot(az);xlabel('采样点');ylabel('加速度/g');title('z轴加速度');
subplot(4,1,4);
plot(a);xlabel('采样点');ylabel('加速度/g');title('加速度幅值');
% subplot(5,1,5);
figure;
plot(Angle);xlabel('采样点');ylabel('角度/°');title('航向角');
axis([0 200 77.2 77.5]);


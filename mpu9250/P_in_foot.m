%%%%%%%%%%%%%%%%%%%%%
%测试pdr程序  V:1.0 [放在脚上的]
%input：3轴加速度和航向角度
%output：步长，步数，和一步的航向
%%%%%%%%%%%%%%%%%%%%%


newData = importdata('./F3/4.tsv', '\t', 2);
data=newData.data;
%2.计算导入的数据的行数(m)和列数(n)
[m, n]=size(data);
%3.输出原始的三轴加速的数据 
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
g=1.0529;
%apsum减去重力以后的加速度
apsum=asum-g;

figure; 
plot(apsum);
title('三轴加速度的和去除重力以后的加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2'); 

%加速的平滑滤波
apsum_e=[];
 for i=1:m-14
     temp=sum(apsum(i:i+14,:))/15;  
     apsum_e = cat(1,apsum_e,temp);
 end
 
%  航向平滑滤波
 dir_p=[];
 for i=1:m-14
     tdirp=sum(data(i:i+14,10))/15;  
     dir_p = cat(1,dir_p,tdirp);
 end

%平滑滤波后的行和列数
[z,q]=size(apsum_e);

step=0;%步数
sflag=0;%步伐峰值标记位
status=1;%当前状态，1为行走状态，2为峰值状态
i0=10;

for i=20:z-20
    
    if apsum_e(i)>0.05
        status=1;
    end
%误判判断
    if status==2&&i-i0<10
        if apsum_e(i)<apsum_e(sflag(step+1))
            sflag(step+1)=i;           
        end
    end 

%stance检测
    if apsum_e(i)<apsum_e(i+1)&&apsum_e(i)<apsum_e(i-1)
%         if apsum_e(i)>(-0.1)&&apsum_e(i)<0&&(apsum_e(i-15)>0.1||apsum_e(i+20)>0.1)&&i-i0>20       
        if apsum_e(i)<0.06&&(apsum_e(i-15)>0.15||apsum_e(i+15)>0.15)&&i-i0>20   
            i0=i;
            status=2;
            step=step+1;          
            sflag=[sflag i];
%             down=0; 
%             climbing=0;
        end
    end    
end
figure;
sflag=sflag(2:step+1);%峰值数组

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dir=0;%方向
for t=1:step-1
    dirb=sflag(t);
    dire=sflag(t+1);
    dir=[dir,sum(dir_p(dirb:dire,:))/(dire-dirb+1)];%假设第9列是航向角
end
dir=dir(2:step);%方向数组

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on;
scatter(sflag,apsum_e(sflag));
plot(apsum_e);
title('平滑滤波后的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');
legend('步伐判定临界点')
step


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%自定义步长为0.6m，实际应用需要更改
plen=zeros(step-1,1);
for i=1:step-1
    plen(i)=1;
end
dir2=zeros(step-1,1);
for j=1:step-1
    dir2(j)=(-dir(j)*pi)/180;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                    PDR定位程序 V：0.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Z=[plen dir2];
% pzero=100;%假设形势过程中的零点个数%%%%%%%%%%%%%%%%%%%%%%%%%大小待定
len=step-1;%仿真步数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x_hat为状态向量，x_hat=[ex,yn,s,f],e为东向坐标，n为北向坐标，s为步长，f为航向
%东北位置坐标的噪声We-N(0,1^2),Wn-N(0,1^2),步长噪声Ws-N(0,0.1^2)(0.1m/s),航向的噪声为Wf-N(0,5^2)(5。)
%Z=[plength dir]为测量值，plength为步长，dir为航向角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ekf 滤波
Qk = diag([1; 1; 0.1; (10*pi)/180])^2;%动态噪声矩阵
Rk = diag([0.1; (5*pi)/180])^2;%观测噪声矩阵

Pk = 5*eye(4);%估计协方差矩阵的初始值，%%%%%%%%%%%%%%%%%%%%%%%大小待定
Pkk_1 = eye(4);%预测协方差矩阵

x_hat = [0,0,0.6,0]';%状态变量的初始值[ex,ny,s,f]%%%%%%%%%%%%%%%%%%%%%%%大小待定
X_est = zeros(len,4);%EKF估计值
x_forecast = zeros(4,1);%预测值
 % 观测矩阵,是个固定值
H=[0,0,1,0;
   0,0,0,1];
for k=1:len
    % 1 状态预测    
    ex = x_hat(1) + x_hat(3)*sin(x_hat(4));
    ny = x_hat(2) + x_hat(3)*cos(x_hat(4));
    s=x_hat(3);
    f=x_hat(4);
    x_forecast = [ex; ny; s; f];%预测值
    % 2  观测预测
    y_yuce=H*x_forecast;
    %  状态矩阵
    F = zeros(4,4);
    F(1,1) = 1; F(1,2) = 0; F(1,3) = sin(x_forecast(4)); F(1,4) = x_forecast(3)*cos(x_forecast(4));
    F(2,1) = 0; F(2,2) = 1; F(2,3)=cos(x_forecast(4)); F(2,4) = -x_forecast(3)*sin(x_forecast(4));
    F(3,1) = 0; F(3,2) = 0; F(3,3) = 1; F(3,4) = 0;
    F(4,1) = 0; F(4,2) = 0; F(4,3) = 0; F(4,4) = 1;
    Pkk_1 = F*Pk*F'+Qk;
    %计算卡尔曼增益
    Kk = Pkk_1*H'*(H*Pkk_1*H'+Rk)^-1; 
    %获取EKF估计值
    x_hat = x_forecast+Kk*(Z(k,:)'-y_yuce);%校正
    %计算状态向量的估计协方差矩阵
    Pk = (eye(4)-Kk*H)*Pkk_1;  
    %将滤波结果保存在矩阵中
    X_est(k,:) = x_hat';
end
%%图形
figure;
plot(X_est(:,1),X_est(:,2), '*g');
xlabel('东向'); 
ylabel('北向'); 
title('EKF simulation');
legend('行走轨迹');


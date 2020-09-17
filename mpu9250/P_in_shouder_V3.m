%%%%%%%%%%%%%%%%%%%%%
%测试pdr程序  V:3.0--[放到肩膀上的]
%input：3轴加速度和航向角度
%output：步长，步数，和一步的航向
%相对于V2合成了航向角误差处理的卡尔曼
%%%%%%%%%%%%%%%%%%%%%
clc
%1.导入数据
newData = importdata('./P1/1.tsv', '\t', 2);
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
% g=1.02649;
g=1.0604;
%apsum减去重力以后的加速度
apsum=asum-g;
figure; 
plot(apsum);
title('三轴加速度的和去除重力以后的加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2'); 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%加速度处理
[N_a,M_a]=size(apsum);
%卡尔曼滤波处理加速度
Q_a = 0.2;%系统噪声
R_a = 0.2;% 测量噪声

% 分配空间
xhat_a=zeros(N_a,1);       % x的后验估计
P_a=0.1;           % 后验方差估计  n*n
xhatminus_a=zeros(N_a,1);  % x的先验估计
Pminus_a=0;      % n*n
K_a=0;   % Kalman增益  n*m

% 估计的初始值都为默认的0，即P=[0 0;0 0],xhat=0
for k = 2:N_a         
    % 时间更新过程
    xhatminus_a(k) = xhat_a(k-1);
    Pminus_a= P_a+Q_a;
    
    % 测量更新过程
    K_a = Pminus_a*inv( Pminus_a+R_a );
    xhat_a(k) = xhatminus_a(k)+K_a*(apsum(k)-xhatminus_a(k));
    P_a = (1-K_a)*Pminus_a;
end
figure; 
plot(xhat_a);
title('卡尔曼滤波后的三轴加速度的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2'); 

% 加速度平滑滤波1
apsum_a1=[];
 for i=1:N_a-10
     temp=sum(xhat_a(i:i+10,:))/11;  
     apsum_a1 = cat(1,apsum_a1,temp);
 end

% figure; 
% plot(apsum_a1);
% title('平滑滤波1后的三轴加速度的和加速度');
% xlabel('单位：采样点');
% ylabel('单位：m/s2');

[P,Q]=size(apsum_a1);
% 加速度平滑滤波2
apsum_a2=[];
 for i=1:P-4
     temp=sum(apsum_a1(i:i+4,:))/5;  
     apsum_a2 = cat(1,apsum_a2,temp);
 end

figure; 
plot(apsum_a2);
title('平滑滤波2后的三轴加速度的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%航向角第一步处理
data(:,10)=-data(:,10);

% 航向平滑滤波1
 dir_p1=[];
 for i=1:m-10
     tdirp1=sum(data(i:i+10,10))/11;  
     dir_p1 = cat(1,dir_p1,tdirp1);
 end
 %航向平滑滤波2
 [V,X]=size(dir_p1);
 dir_p2=[];
 for i=1:V-4
     tdirp2=sum(dir_p1(i:i+4,:))/5;  
     dir_p2 = cat(1,dir_p2,tdirp2);
 end
 
 
figure; 
plot(dir_p2);
title('平滑滤波2后的航向角');
xlabel('单位：采样点');
ylabel('单位：°');
 
 
 
 %平滑滤波后的加速度 行和列数
[Z,O1]=size(apsum_a2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ptemp=0;%步长中间值
k=1;%步长模型因子%%%%%%%%%%%%%%%%%%%%%%%%%%%待测定
plength=0;%步长
pzero=0;%零点计数
pb=0;%一步开始的时刻
pmax=0;%记录峰值点的值
pflag=0;%标记为零点的开始
tflag=0;
pclimbing=0;%上升标记
min=0.15;%一步中的加速度最小值
max_a=-0.1;%一步中的加速度最大值
step=0;%步数
sflag=0;%步伐峰值标记位
climbing=0;%上升标记
status=1;%当前状态，1为行走状态，2为峰值状态
down=0;%下降标记

for i=2:Z-1
    if apsum_a2(i)>max_a
        max_a=apsum_a2(i);
    end
    if apsum_a2(i)<min
        min=apsum_a2(i);
    end
    if apsum_a2(i)>apsum_a2(i-1)
        climbing=climbing+1;
    end
    if apsum_a2(i)<apsum_a2(i-1)
        down=down+1;
    end
    if down>10
        status=1;
    end
    if status==2&&i-i0<8
        if apsum_a2(i)>apsum_a2(sflag(step+1))
            sflag(step+1)=i;
            pmax=apsum_a2(i);
        end
    end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%     零点交叉  
    if pflag==1&&apsum_a2(tflag+8)<0.025
        pflag=0; 
    end

    if pflag==1&&apsum_a2(i)>apsum_a2(i-1)
        pclimbing=pclimbing+1;
    end
       
    %用来判断一步的开始或结束
    if pflag==1&&pclimbing>5&&apsum_a2(tflag+pclimbing)>0.015
        pb=[pb tflag];%一步开始的时刻 
        ptemp=k*(pmax-min)^(1/4);%获取步长
        plength=[plength ptemp];%步长数组
        pflag=0; 
        pzero=pzero+1;
    end
    
    if apsum_a2(i)>0&&apsum_a2(i-1)<0
        tflag=i;%记下当前的零点时刻
        pflag=1;%标记为零点的开始
        pclimbing=0;%零点的开始以后上升步的点数      
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%峰值检测
    if apsum_a2(i)>apsum_a2(i+1)&&apsum_a2(i)>apsum_a2(i-1)
%         if max_a>0.045&&climbing>10&&apsum_e(i)>0.04
        if apsum_a2(i)>0.045&&climbing>8
            i0=i;
            pmax=apsum_a2(i);
            status=2;
            step=step+1;          
%             max_a=min;
            min=apsum_a2(i);
            sflag=[sflag i];
            climbing=0;
            down=0;
        end
    end    
end
figure;
sflag=sflag(2:step+1);%峰值数组
pb=pb(2:pzero+1);%零点数组

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dir=0;%航向
for t=1:pzero-1
    dirb=pb(t);
    dire=pb(t+1);
    dir=[dir,sum(dir_p2(dirb:dire,:))/(dire-dirb+1)];%取一步内航向角的平均值
end
dir=dir(2:pzero);%方向数组
dir=[dir dir(pzero-1)]';%最后一步的方向跟上一次方向一致

plength=plength(3:pzero+1);%步长数组，第一步的步长不能要，要舍弃
plength=[plength plength(pzero-1)]';%最后一步步长跟上一步一致
%最后一步结束的零点无法探测到，所以令最后一步的步长等于上一步，
%最后一步的方向等于上一步。

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on;
scatter(sflag,apsum_a2(sflag));
scatter(pb,apsum_a2(pb));
plot(apsum_a2);
title('加速度的步伐和零点检测');
xlabel('单位：采样点');
ylabel('单位：m/s2');
legend('步伐判定临界点')
pzero
step

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% iHDE算法反馈航向角

[N_d,O2]=size(dir);
dir_arr=zeros(1,3);
max_dir=90;%航向最大偏差
error=zeros(N_d,1);%角度误差,大于0往左偏，小于0往右偏。航向角减去误差值
for k=4:N_d
    E=0;
    SLP=0;%直行标记
    %直行判断
    for t=1:3
        dir_arr(t)=abs(dir(k-t)-mean(dir(k-3:k)));
    end
    max_dir=max(dir_arr);
    if(max_dir<15) 
        SLP=1;
    end
   
    if SLP==1
      %直行情况下，航向角的观测值值等于三步内航向角的平均值
      %航向角的误差值等于当前航向角的值减去航向角的观测值
      error(k)=dir(k)-mean(dir(k-3:k));  
    end
    
   %主方向判断
   if dir(k)>0||dir(k)==0  %航向角是正值
    E=45-mod(dir(k),90);
   end
   if dir(k)<0             %航向角是负值
    E=-45-mod(dir(k),-90);
   end
   
    if (abs(E)>15||abs(E)==30)&&SLP==1%说明是主方向上直行,取观测值为主方向的值，将原先的误差值覆盖
        if dir(k)>-15&&dir(k)<15
            error(k)=dir(k)-0;
        end
        if dir(k)>75&&dir(k)<105
            error(k)=dir(k)-90;
        end
        if dir(k)>165&&dir(k)<180
            error(k)=dir(k)-180;
        end
        if dir(k)>-180&&dir(k)<-150   %原先设置的是-165
            error(k)=dir(k)+180;
        end
        if dir(k)>-105&&dir(k)<-70   %
            error(k)=dir(k)+90;
        end      
    end  
end

%将航向误差值输入到卡尔曼滤波中

dir_R=zeros(N_d,1);%置信度因子，作为航向角误差处理的观测噪声
for q=1:N_d
    dir_R(q)=0.1/exp(-5*abs(error(q))/90);
end
% dir_Q = 0.2;%系统噪声
% % dir_R = 0.2;% 测量噪声
% 
% dir_xhat=zeros(N_d,1);       % x的后验估计
% dir_P=0.1;           % 后验方差估计  n*n
% dir_xhatminus=zeros(N_d,1);  % x的先验估计
% dir_Pminus=0;      % n*n
% dir_K=0;   % Kalman增益  n*m


% 估计的初始值都为默认的0，即P=[0 0;0 0],xhat=0
% for r = 2:N_d         
%     % 时间更新过程
%     dir_xhatminus(r) = dir_xhat(r-1);
%     dir_Pminus= dir_P+dir_Q;
%     
%     % 测量更新过程
%     dir_K = dir_Pminus*inv( dir_Pminus+dir_R(r) );
%     dir_xhat(r) = dir_xhatminus(r)+dir_K*(error(r)-dir_xhatminus(r));
%     dir_P = (1-dir_K)*dir_Pminus;  
% end
% dir_cor=zeros(N_d,1);%校准之后的航向角
% for j=1:N_d
%     %航向纠正，航向值减去误差值
%     dir_cor(j)=dir(j)-dir_xhat(j);
% end
% 
figure; 
plot(dir);
title('步伐检测后的原始航向角');
xlabel('单位：采样点');
ylabel('单位：m/s2');
% 
% figure; 
% plot(dir_cor);
% title('iHDE算法处理后的航向角');
% xlabel('单位：采样点');
% ylabel('单位：m/s2'); 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%自定义步长为0.6m，实际应用需要更改
plen=zeros(pzero,1);
for i=1:pzero
    plen(i)=0.6;
end
%航向变成弧度
dir2=zeros(pzero,1);
error2=zeros(pzero,1);%航向角误差变弧度值
for j=1:pzero
    error2(j)=(error(j)*pi)/180;
    dir2(j)=(dir(j)*pi)/180;  
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                    PDR定位程序 V：0.1
%%%%%%%%%%%%%%%%%%%%%%%%%%%    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Z=[plen dir2,error2];
% pzero=100;%假设形势过程中的零点个数%%%%%%%%%%%%%%%%%%%%%%%%%大小待定
len=pzero;%仿真步数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x_hat为状态向量，x_hat=[ex,yn,s,f],e为东向坐标，n为北向坐标，s为步长，f为航向
%东北位置坐标的噪声We-N(0,1^2),Wn-N(0,1^2),步长噪声Ws-N(0,0.1^2)(0.1m/s),航向的噪声为Wf-N(0,5^2)(5。)
%Z=[plength dir]为测量值，plength为步长，dir为航向角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ekf 滤波
Qk = diag([1; 1; 0.1; (5*pi)/180;(5*pi)/180])^2;%动态噪声矩阵

%观测噪声矩阵
Rk=cell(len,1);
for v=1:len
    Rk{v,1}=diag([0.1; (5*pi)/180;dir_R(v)])^2;
end

Pk = 2*eye(5);%估计协方差矩阵的初始值，%%%%%%%%%%%%%%%%%%%%%%%大小待定
Pkk_1 = eye(5);%预测协方差矩阵

x_hat = [0,0,0.6,0,0]';%状态变量的初始值[ex,ny,s,f]%%%%%%%%%%%%%%%%%%%%%%%大小待定
X_est = zeros(len,5);%EKF估计值
x_forecast = zeros(5,1);%预测值
 % 观测矩阵,是个固定值
H=[0,0,1,0,0;
   0,0,0,1,0;
   0,0,0,0,1];
for k=1:len
    % 1 状态预测    
    s=x_hat(3);
    f=x_hat(4)-x_hat(5);
    e=x_hat(5);
    ex = x_hat(1) + s*sin(f);
    ny = x_hat(2) + s*cos(f);
    x_forecast = [ex; ny; s; f;e];%预测值
    % 2  观测预测
    y_yuce=H*x_forecast;
    %  状态矩阵
    F = zeros(5,5);
    F(1,1) = 1; F(1,2) = 0; F(1,3) = sin(x_forecast(4)); F(1,4) = x_forecast(3)*cos(x_forecast(4));F(1,5)=0;
    F(2,1) = 0; F(2,2) = 1; F(2,3)=cos(x_forecast(4)); F(2,4) = -x_forecast(3)*sin(x_forecast(4));F(2,4)=0;
    F(3,1) = 0; F(3,2) = 0; F(3,3) = 1; F(3,4) = 0;F(3,5) = 0;
    F(4,1) = 0; F(4,2) = 0; F(4,3) = 0; F(4,4) = 1;F(4,5)=-1;
    F(5,1)=0;F(5,1)=0;F(5,1)=0;F(5,1)=0;F(5,1)=0;F(5,1)=1;
    Pkk_1 = F*Pk*F'+Qk;
    %计算卡尔曼增益
    Kk = Pkk_1*H'*(H*Pkk_1*H'+Rk{k,1})^-1; 
    %获取EKF估计值
    x_hat = x_forecast+Kk*(Z(k,:)'-y_yuce);%校正
    %计算状态向量的估计协方差矩阵
    Pk = (eye(5)-Kk*H)*Pkk_1;  
    %将滤波结果保存在矩阵中
    X_est(k,:) = x_hat';
end
%%图形
figure; 
plot(X_est(:,4));
title('iHDE算法处理后的航向角');
xlabel('单位：采样点');
ylabel('单位：m/s2'); 


figure;
plot(X_est(:,1),X_est(:,2), '*g');
% axis([-0.4 0.4 0 30]);
xlabel('东向'); 
ylabel('北向'); 
title('EKF simulation');
legend('行走轨迹');


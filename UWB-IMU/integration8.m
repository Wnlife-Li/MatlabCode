
clc;
clear all;

%%%%%%%%%%%%%%-------------   【实测实验代码】    基于UWB和IMU的室内定位融合算法--------------%%%%%%%%%%%%%%%%%

%----1.惯性导航的数据

%%%---1.1 4个基站的坐标
x1=[9 10.8];
x2=[9 0.75];
x3=[3 0.75];
x4=[3 10.8];
bs=[x1;x2;x3;x4];

%%%---1.2 航向信息
h=zeros(35,1);
for i=1:12
    h(i)=-180;
end
for i=13:22
    h(i)=-90;
end
for i=23:35
    h(i)=0;
end

% 给航向角加入噪声
% figure;
% plot(h,'-*g');
% hold on;
h=awgn(h,-10);%给航向角加入高斯白噪声
% plot(h,'-or');

%将航向变为弧度数
for i=1:35
   h(i)=(h(i)*pi)/180;
end
%%%--- 1.4 步数 
N_P=35;
%%%--- 1.5 步长
N_S=zeros(35,1);
for i=1:35
    N_S(i)=0.6;
end
% figure;
% plot(N_S,'-*g');
% hold on;
N_S=awgn(N_S,30);%给步长加入高斯白噪声
% plot(N_S,'-or');

%实际的行走坐标
r_p=importdata('F:\Matlab\R2016\bin\UWB-IMU\TestData2\r_p.txt');
P_P=r_p;

%%% UWB的测量距离【四个基站到标签的距离】
data = importdata('F:\Matlab\R2016\bin\UWB-IMU\TestData2\r_x.txt');
%计算导入的数据的行数(m)和列数(n)
[m, n]=size(data);

d_UWB_1=data(:,1);
d_UWB_2=data(:,2);
d_UWB_3=data(:,3);
d_UWB_4=data(:,4);

len=N_P;%仿真步数
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【自定义】数据融合算法%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

              %%%%%%%%%%%%%%%%%%%%%%    【融合部分EKF数据的初始化】    %%%%%%%%%%%%%%%%%%%%
               
% %假设d为第n步标签到基站的距离  ---- 有4个基站
d=data;
residual=zeros(4,1);
Z_R=[d_UWB_1,d_UWB_2,d_UWB_3,d_UWB_4,N_S,h]; %  -----  融合EKF的测量值 （6个距离，一个步长和一个航向） n行8列
HH_R=zeros(6,4);
X_est_R = zeros(len,4);     %----  融合的EKF最优估计值
X_est_R(1,:)=[9,10.2,0.6,(-180)*pi/180];
P_R=0.1*eye(4);%测量协方差矩阵的初始值
P_R(4,4)=P_R(4,4)*40;
R_F = diag([0.5; 0.5; 0.5; 0.5; 0.1; (0.1*pi)/180])^2;  %【融合】观测噪声矩阵
x_R=[9,10.2,0.6,(-180)*pi/180]';%状态变量的初始值
% R_R_R=zeros(len,4);
% d_t_1=zeros(4,1);
Threshold=0.4; %%%%-----NLOS检测

%%%%%%%%%%%%%%%%%%%%%%%%%%%%       【惯性导航的EKF的初始化】    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x_hat为状态向量，x_hat=[ex,yn,s,f],e为东向坐标，n为北向坐标，s为步长，f为航向
%东北位置坐标的噪声We-N(0,1^2),Wn-N(0,1^2),步长噪声Ws-N(0,0.1^2)(0.1m/s),航向的噪声为Wf-N(0,5^2)(5。)
%Z=[plength dir]为测量值，plength为步长，dir为航向角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 惯性导航使用ekf 得出当前位置
Qk = 0.5*diag([0.5; 0.5; 0.1; (5*pi)/180])^2;%预测噪声矩阵
Rk = 0.01*diag([0.1; (5*pi)/180])^2;%测量噪声矩阵       
Z=[N_S,h];  %惯性导航的测量值（一个步长一个航向）
Pk = 2*eye(4);%测量协方差矩阵的初始值，%%%%%%%%%%%%%%%%%%%%%%%大小待定
Pkk_1 = eye(4);%预测协方差矩阵
x_hat = [9,10.2,0.6,(-180)*pi/180]';%状态变量的初始值[ex,ny,s,f]%%%%%%%%%%%%%%%%%%%%%%%大小待定
 % 观测矩阵,是个固定值
H=[0,0,1,0;
   0,0,0,1];

for k=2:len
   
    %%%%%%%%%%%%%%%%%%%           【惯性导航部分的EKF】        %%%%%%%%%%%%%%%%%%
    % 1 状态预测    
    ex_R = x_R(1) + x_R(3)*sin(x_R(4));
    ny_R = x_R(2) + x_R(3)*cos(x_R(4));
    s_R=x_R(3);
    f_R=x_R(4);
    x_forecast_R = [ex_R; ny_R; s_R; f_R];%预测值
    % 2  观测预测
    y_yuce=H*x_forecast_R;
    %  状态矩阵
    F_R = zeros(4,4);
    F_R (1,1) = 1; F_R (1,2) = 0; F_R (1,3) = sin(x_forecast_R(4)); F_R (1,4) = x_forecast_R(3)*cos(x_forecast_R(4));
    F_R (2,1) = 0; F_R (2,2) = 1; F_R (2,3)=cos(x_forecast_R(4)); F_R (2,4) = -x_forecast_R(3)*sin(x_forecast_R(4));
    F_R (3,1) = 0; F_R (3,2) = 0; F_R (3,3) = 1; F_R (3,4) = 0;
    F_R (4,1) = 0; F_R (4,2) = 0; F_R (4,3) = 0; F_R (4,4) = 1;    
    
    Pkk_1 = F_R*Pk*F_R'+Qk;%惯导的预测协方差矩阵   
%     计算卡尔曼增益
    Kk = Pkk_1*H'*(H*Pkk_1*H'+Rk)^-1; 
    %获取EKF估计值
    x_hat = x_forecast_R+Kk*(Z(k,:)'-y_yuce);%校正
    %计算状态向量的估计协方差矩阵
    Pk = (eye(4)-Kk*H)*Pkk_1;  
    
    %%%%%%%%%%%%%%%%%%%%%%%%%       【UWB和惯性导航融合部分的EKF】      %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Pkk_R = F_R *P_R*F_R'+Qk;     %【融合的预测协方差矩阵】
        
    %计算测量矩阵的雅各比矩阵     ------（b_e,b_n）基站的坐标
    HH_R(1,1)=(x_forecast_R(1)-x1(1))*((x1(1)-x_forecast_R(1))^2 +(x1(2)-x_forecast_R(1))^2)^(-0.5);
    HH_R(1,2)=(x_forecast_R(2)-x1(2))*((x1(1)-x_forecast_R(1))^2 +(x1(2)-x_forecast_R(1))^2)^(-0.5);
    
    HH_R(2,1)=(x_forecast_R(1)- x2(1))*((x_forecast_R(1)- x2(1))^2 +(x_forecast_R(1)-x2(2))^2)^(-0.5);
    HH_R(2,2)=(x_forecast_R(2)- x2(2))*((x_forecast_R(1)- x2(1))^2 +(x_forecast_R(1)-x2(2))^2)^(-0.5); 
    
    HH_R(3,1)=(x_forecast_R(1)- x3(1))*((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(1)- x3(2))^2)^(-0.5);
    HH_R(3,2)=(x_forecast_R(2)- x3(2))*((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(1)- x3(2))^2)^(-0.5);
    
    HH_R(4,1)=(x_forecast_R(1)- x4(1))*((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(1)- x4(2))^2)^(-0.5);
    HH_R(4,2)=(x_forecast_R(2)- x4(2))*((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(1)- x4(2))^2)^(-0.5);
     
    HH_R(5,3)=1; 
    HH_R(6,4)=1; 
    
    %预测值到测量值
    y_R(1,1)=sqrt((x1(1)-  x_forecast_R(1))^2 +(x1(2)-x_forecast_R(2))^2);
    y_R(2,1)=sqrt((x_forecast_R(1)-  x2(1))^2 +(x_forecast_R(2)- x2(2))^2);
    y_R(3,1)=sqrt((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(2)- x3(2))^2);
    y_R(4,1)=sqrt((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(2)- x4(2))^2);
    y_R(5,1)=x_forecast_R(3);
    y_R(6,1)=x_forecast_R(4);
    
    %%%把当前时刻的惯性导航的位置估计值转换为   距离值
    y_R_F(1,1)=sqrt((x1(1)-  x_hat(1))^2 +(x1(2)-x_hat(2))^2);
    y_R_F(2,1)=sqrt((x_hat(1)-  x2(1))^2 +(x_hat(2)- x2(2))^2);
    y_R_F(3,1)=sqrt((x_hat(1)-  x3(1))^2 +(x_hat(2)- x3(2))^2);
    y_R_F(4,1)=sqrt((x_hat(1)-  x4(1))^2 +(x_hat(2)- x4(2))^2);
    
    %%%---NLOS检测---%%%  
    r_d=abs(d(k,:)' - y_R_F);
    residual=ones(4,1);
    for i=1:4
        if(r_d(i)>Threshold)
            Temp=r_d(i)-Threshold;
            residual(i)=Temp*10000;  %%计算残差
        end
    end
    de=diag([residual(1); residual(2); residual(3); residual(4); 1; 1]);%融合噪声的矩阵的残差系数
    R_E=R_F*de;   %----  融合噪声的矩阵
    %计算卡尔曼增益
    K_R = Pkk_R*HH_R'*(HH_R*Pkk_R*HH_R'+R_E)^-1; 
    %最优估计值
    x_R = x_forecast_R+K_R*(Z_R(k,:)'-y_R);%校正
    %计算状态向量的估计协方差矩阵
    P_R = (eye(4)-K_R*HH_R)*Pkk_R;  
    %将滤波结果保存在矩阵中，融合后的最优估计值
    X_est_R(k,:) = x_R';  
    
end
% figure;
% %测试自定义算法的效果
% plot(X_est_R(:,1),X_est_R(:,2),'g-*','MarkerSize',12,'LineWidth',1.1);




%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据融合算法【三角形】%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

              %%%%%%%%%%%%%%%%%%%%%%    【融合部分EKF数据的初始化】    %%%%%%%%%%%%%%%%%%%%
               
% %假设d为第n步标签到基站的距离  ---- 有4个基站
d=data;
% residual=zeros(6,1);
Z_R=[d_UWB_1, d_UWB_2, d_UWB_3, d_UWB_4, N_S, h]; %  -----  融合EKF的测量值 （4个距离，一个步长和一个航向） 
HH_R=zeros(6,4);
X_est_R_I = zeros(len,4);     %----  融合的EKF最优估计值
X_est_R_I(1,:)=[9, 10.2, 0.6, (-180)*pi/180];
P_R=1*eye(4);%测量协方差矩阵的初始值
% P_R(4,4)=P_R(4,4)*40;
R_F = diag([0.5; 0.5; 0.5; 0.5; 0.1; (6*pi)/180])^2;  %【融合】观测噪声矩阵
x_R=[9,10.2,0.6,(-180)*pi/180]';%状态变量的初始值
R_R_R=zeros(len, 4);
d_t_1=zeros(4, 1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%       【惯性导航的EKF的初始化】    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x_hat为状态向量，x_hat=[ex,yn,s,f],e为东向坐标，n为北向坐标，s为步长，f为航向
%东北位置坐标的噪声We-N(0,1^2),Wn-N(0,1^2),步长噪声Ws-N(0,0.1^2)(0.1m/s),航向的噪声为Wf-N(0,5^2)(5。)
%Z=[plength dir]为测量值，plength为步长，dir为航向角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 惯性导航使用ekf 得出当前位置
Qk = 0.5*diag([0.5; 0.5; 0.1; (5*pi)/180])^2;%动态噪声矩阵
Rk = 0.01*diag([0.1; (5*pi)/180])^2;%观测噪声矩阵       
Z=[N_S,h];  %惯性导航的测量值（一个步长一个航向）
Pk = 1*eye(4);%测量协方差矩阵的初始值，%%%%%%%%%%%%%%%%%%%%%%%大小待定
Pkk_1 = eye(4);%预测协方差矩阵
x_hat = [9, 10.2, 0.6, (-180)*pi/180]';%状态变量的初始值[ex,ny,s,f]%%%%%%%%%%%%%%%%%%%%%%%大小待定
 % 观测矩阵,是个固定值
H=[0,0,1,0;
   0,0,0,1];

for k=2:len
   
    %%%%%%%%%%%%%%%%%%%           【惯性导航部分的EKF】        %%%%%%%%%%%%%%%%%%
    % 1 状态预测    
    ex_R = x_R(1) + x_R(3)*sin(x_R(4));
    ny_R = x_R(2) + x_R(3)*cos(x_R(4));
    s_R=x_R(3);
    f_R=x_R(4);
    x_forecast_R = [ex_R; ny_R; s_R; f_R];%预测值
    % 2  观测预测
    y_yuce=H*x_forecast_R;
    %  状态矩阵
    F_R = zeros(4,4);
    F_R (1,1) = 1; F_R (1,2) = 0; F_R (1,3) = sin(x_forecast_R(4)); F_R (1,4) = x_forecast_R(3)*cos(x_forecast_R(4));
    F_R (2,1) = 0; F_R (2,2) = 1; F_R (2,3)=cos(x_forecast_R(4)); F_R (2,4) = -x_forecast_R(3)*sin(x_forecast_R(4));
    F_R (3,1) = 0; F_R (3,2) = 0; F_R (3,3) = 1; F_R (3,4) = 0;
    F_R (4,1) = 0; F_R (4,2) = 0; F_R (4,3) = 0; F_R (4,4) = 1;    
    Pkk_1 = F_R*Pk*F_R'+Qk;%惯导的预测协方差矩阵   
%     计算卡尔曼增益
    Kk = Pkk_1*H'*(H*Pkk_1*H'+Rk)^-1; 
    %获取EKF估计值
    x_hat = x_forecast_R+Kk*(Z(k,:)'-y_yuce);%校正
    %计算状态向量的估计协方差矩阵
    Pk = (eye(4)-Kk*H)*Pkk_1;  
      
    %%%%%%%%%%%%%%%%%       【UWB和惯性导航融合部分的EKF】      %%%%%%%%%%%%%%%%%%%%%%%%
    Pkk_R = F_R *P_R*F_R'+Qk;     %【融合的预测协方差矩阵】
    
    %使用惯性导航得到的当前位置坐标算出行人移动的距离  msd------------????????????上一个时刻的位置应该用融合算出来的位置
    msd=sqrt((x_hat(1)-X_est_R_I(k-1,1))^2+(x_hat(2)-X_est_R_I(k-1,2))^2);
    %计算前一个时刻基站到移动节点的距离
    for g=1:4
        d_t_1(g)=sqrt((X_est_R_I(k-1,1)-bs(g,1))^2+(X_est_R_I(k-1,2)-bs(g,2))^2);%%---????上一个时刻的位置应该用融合算出来的位置
    end
    %计算残差
    residual=ones(4,1);
    for g=1:4                          %-----此处假设有4个基站
%       residual(g)=abs(msd-abs(d(k-1,g)-d(k,g))-Threshold)*k_r;   %Threshold=-1待定  ------   k_r为残差到构造观测矩阵的一个调节系数   
        R_R_R(k,g)= msd-abs(d_t_1(g)-d(k,g));  
        if(R_R_R(k,g)<0.3) 
            residual(g)=1000000*abs(R_R_R(k,g));
        end
    end
    %计算融合时需要的测量噪声矩阵
    de=diag([residual(1); residual(2); residual(3); residual(4); 1; 1]);%融合噪声的矩阵的残差系数
    R_E=R_F*de;   %----  融合噪声的矩阵
    %计算测量矩阵的雅各比矩阵     ------（b_e,b_n）基站的坐标
    HH_R(1,1)=(x_forecast_R(1)-x1(1))*((x1(1)-x_forecast_R(1))^2 +(x1(2)-x_forecast_R(1))^2)^(-0.5);
    HH_R(1,2)=(x_forecast_R(2)-x1(2))*((x1(1)-x_forecast_R(1))^2 +(x1(2)-x_forecast_R(1))^2)^(-0.5);
    
    HH_R(2,1)=(x_forecast_R(1)- x2(1))*((x_forecast_R(1)- x2(1))^2 +(x_forecast_R(1)-x2(2))^2)^(-0.5);
    HH_R(2,2)=(x_forecast_R(2)- x2(2))*((x_forecast_R(1)- x2(1))^2 +(x_forecast_R(1)-x2(2))^2)^(-0.5); 
    
    HH_R(3,1)=(x_forecast_R(1)- x3(1))*((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(1)- x3(2))^2)^(-0.5);
    HH_R(3,2)=(x_forecast_R(2)- x3(2))*((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(1)- x3(2))^2)^(-0.5);
    
    HH_R(4,1)=(x_forecast_R(1)- x4(1))*((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(1)- x4(2))^2)^(-0.5);
    HH_R(4,2)=(x_forecast_R(2)- x4(2))*((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(1)- x4(2))^2)^(-0.5);
       
    HH_R(5,3)=1; 
    HH_R(6,4)=1; 
    
    %预测值到测量值
    y_R(1,1)=sqrt((x1(1)-  x_forecast_R(1))^2 +(x1(2)-x_forecast_R(2))^2);
    y_R(2,1)=sqrt((x_forecast_R(1)-  x2(1))^2 +(x_forecast_R(2)- x2(2))^2);
    y_R(3,1)=sqrt((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(2)- x3(2))^2);
    y_R(4,1)=sqrt((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(2)- x4(2))^2);
    y_R(5,1)=x_forecast_R(3);
    y_R(6,1)=x_forecast_R(4);
    
    %计算卡尔曼增益
    K_R = Pkk_R*HH_R'*(HH_R*Pkk_R*HH_R'+R_E)^-1; 
    %最优估计值
    x_R = x_forecast_R+K_R*(Z_R(k,:)'-y_R);%校正
    %计算状态向量的估计协方差矩阵
    P_R = (eye(4)-K_R*HH_R)*Pkk_R;  
    %将滤波结果保存在矩阵中，融合后的最优估计值
    X_est_R_I(k,:) = x_R';  
    
end

% figure;
% plot(X_est_R_I(:,1),X_est_R_I(:,2),'m-d','MarkerSize',9,'LineWidth',1.1);


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据融合算法【UWB和INS无NLOS检测算法】%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

              %%%%%%%%%%%%%%%%%%%%%%    【融合部分EKF数据的初始化】    %%%%%%%%%%%%%%%%%%%%
               
% %假设d为第n步标签到基站的距离  ---- 假设有4个基站
d=data;
% residual=zeros(6,1);
Z_R=[d_UWB_1,d_UWB_2,d_UWB_3,d_UWB_4,N_S,h]; %  -----  融合EKF的测量值 （6个距离，一个步长和一个航向） n行8列
HH_R=zeros(6,4);
X_est_R_I_U = zeros(len,4);     %----  融合的EKF最优估计值
X_est_R_I_U(1,:)=[9, 10.2, 0.6, (-180)*pi/180];
P_R=1*eye(4);%测量协方差矩阵的初始值
% P_R(4,4)=P_R(4,4)*40;
R_F = diag([0.5; 0.5; 0.5; 0.5; 0.1; (0.5*pi)/180])^2;  %【融合】观测噪声矩阵
x_R=[9, 10.2, 0.6, (-180)*pi/180]';%状态变量的初始值
R_R_R=zeros(len,4);
d_t_1=zeros(4,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%       【惯性导航的EKF的初始化】    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x_hat为状态向量，x_hat=[ex,yn,s,f],e为东向坐标，n为北向坐标，s为步长，f为航向
%东北位置坐标的噪声We-N(0,1^2),Wn-N(0,1^2),步长噪声Ws-N(0,0.1^2)(0.1m/s),航向的噪声为Wf-N(0,5^2)(5。)
%Z=[plength dir]为测量值，plength为步长，dir为航向角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 惯性导航使用ekf 得出当前位置
Qk = 0.5*diag([0.5; 0.5; 0.1; (5*pi)/180])^2;%动态噪声矩阵
Rk = 0.01*diag([0.1; (5*pi)/180])^2;%观测噪声矩阵       
Z=[N_S,h];  %惯性导航的测量值（一个步长一个航向）
Pk = 1*eye(4);%测量协方差矩阵的初始值，%%%%%%%%%%%%%%%%%%%%%%%大小待定
Pkk_1 = eye(4);%预测协方差矩阵
x_hat = [9, 10.2, 0.6, (-180)*pi/180]';%状态变量的初始值[ex,ny,s,f]%%%%%%%%%%%%%%%%%%%%%%%大小待定
 % 观测矩阵,是个固定值
H=[0,0,1,0;
   0,0,0,1];

for k=2:len
   
    %%%%%%%%%%%%%%%%%%%           【惯性导航部分的EKF】        %%%%%%%%%%%%%%%%%%
    % 1 状态预测    
    ex_R = x_R(1) + x_R(3)*sin(x_R(4));
    ny_R = x_R(2) + x_R(3)*cos(x_R(4));
    s_R=x_R(3);
    f_R=x_R(4);
    x_forecast_R = [ex_R; ny_R; s_R; f_R];%预测值
    % 2  观测预测
    y_yuce=H*x_forecast_R;
    %  状态矩阵
    F_R = zeros(4,4);
    F_R (1,1) = 1; F_R (1,2) = 0; F_R (1,3) = sin(x_forecast_R(4)); F_R (1,4) = x_forecast_R(3)*cos(x_forecast_R(4));
    F_R (2,1) = 0; F_R (2,2) = 1; F_R (2,3)=cos(x_forecast_R(4)); F_R (2,4) = -x_forecast_R(3)*sin(x_forecast_R(4));
    F_R (3,1) = 0; F_R (3,2) = 0; F_R (3,3) = 1; F_R (3,4) = 0;
    F_R (4,1) = 0; F_R (4,2) = 0; F_R (4,3) = 0; F_R (4,4) = 1;    
    Pkk_1 = F_R*Pk*F_R'+Qk;%惯导的预测协方差矩阵   
%     计算卡尔曼增益
    Kk = Pkk_1*H'*(H*Pkk_1*H'+Rk)^-1; 
    %获取EKF估计值
    x_hat = x_forecast_R+Kk*(Z(k,:)'-y_yuce);%校正
    %计算状态向量的估计协方差矩阵
    Pk = (eye(4)-Kk*H)*Pkk_1;  
    
    
    %%%%%%%%%%%%%%%%%       【UWB和惯性导航融合部分的EKF】      %%%%%%%%%%%%%%%%%%%%%%%%
    Pkk_R = F_R *P_R*F_R'+Qk;     %【融合的预测协方差矩阵】
    
    R_E=R_F;   %----  融合噪声的矩阵
    %计算测量矩阵的雅各比矩阵     ------（b_e,b_n）基站的坐标
    HH_R(1,1)=(x_forecast_R(1)-x1(1))*((x1(1)-x_forecast_R(1))^2 +(x1(2)-x_forecast_R(1))^2)^(-0.5);
    HH_R(1,2)=(x_forecast_R(2)-x1(2))*((x1(1)-x_forecast_R(1))^2 +(x1(2)-x_forecast_R(1))^2)^(-0.5);
    
    HH_R(2,1)=(x_forecast_R(1)- x2(1))*((x_forecast_R(1)- x2(1))^2 +(x_forecast_R(1)-x2(2))^2)^(-0.5);
    HH_R(2,2)=(x_forecast_R(2)- x2(2))*((x_forecast_R(1)- x2(1))^2 +(x_forecast_R(1)-x2(2))^2)^(-0.5); 
    
    HH_R(3,1)=(x_forecast_R(1)- x3(1))*((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(1)- x3(2))^2)^(-0.5);
    HH_R(3,2)=(x_forecast_R(2)- x3(2))*((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(1)- x3(2))^2)^(-0.5);
    
    HH_R(4,1)=(x_forecast_R(1)- x4(1))*((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(1)- x4(2))^2)^(-0.5);
    HH_R(4,2)=(x_forecast_R(2)- x4(2))*((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(1)- x4(2))^2)^(-0.5);
    
    HH_R(5,3)=1; 
    HH_R(6,4)=1; 
    
    %预测值到测量值
    y_R(1,1)=sqrt((x1(1)-  x_forecast_R(1))^2 +(x1(2)-x_forecast_R(2))^2);
    y_R(2,1)=sqrt((x_forecast_R(1)-  x2(1))^2 +(x_forecast_R(2)- x2(2))^2);
    y_R(3,1)=sqrt((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(2)- x3(2))^2);
    y_R(4,1)=sqrt((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(2)- x4(2))^2);
    y_R(5,1)=x_forecast_R(3);
    y_R(6,1)=x_forecast_R(4);
    
    %计算卡尔曼增益
    K_R = Pkk_R*HH_R'*(HH_R*Pkk_R*HH_R'+R_E)^-1; 
    %最优估计值
    x_R = x_forecast_R+K_R*(Z_R(k,:)'-y_R);%校正
    %计算状态向量的估计协方差矩阵
    P_R = (eye(4)-K_R*HH_R)*Pkk_R;  
    %将滤波结果保存在矩阵中，融合后的最优估计值
    X_est_R_I_U(k,:) = x_R';  
    
end

% figure;
% plot(X_est_R_I_U(:,1),X_est_R_I_U(:,2),'b-s','MarkerSize',8,'LineWidth',1.1);
%%

% %%正常残差检验值
% figure;
% subplot(2,1,1);
% plot(R_R_R(:,1));
% subplot(2,1,2);
% plot(R_R_R(:,2));
% figure;
% subplot(2,1,1);
% plot(R_R_R(:,5));
% subplot(2,1,2);
% plot(R_R_R(:,6));
% %%%加了噪声的残差检验值
% figure;
% subplot(2,1,1);
% scatter(R_UWB_3_U,R_R_R(R_UWB_3_U,3),'gO');
% hold on;
% plot(R_R_R(:,3));
% subplot(2,1,2);
% scatter(R_UWB_4_U,R_R_R(R_UWB_4_U,4),'gO');
% hold on;
% plot(R_R_R(:,4));


% % % ----描述行走过程中的真实轨迹
tx=9;
ty=10.2;
l=6;
w=7.2;
x=[tx,tx,tx-l,tx-l];
y=[ty,ty-w,ty-w,ty];
figure;
plot(x,y,'k','LineWidth',1.5);
hold on
%%%惯性导航的轨迹
% plot(X_est(:,1),X_est(:,2),'g-*','MarkerSize',8);
%%UBW和INS无NLOS检测算法
plot(X_est_R_I_U(:,1),X_est_R_I_U(:,2),'b-s','MarkerSize',8,'LineWidth',1.1);
%%%三角形算法的轨迹
plot(X_est_R_I(:,1),X_est_R_I(:,2),'m-d','MarkerSize',9,'LineWidth',1.1);
%%%自定义算法的轨迹
plot(X_est_R(:,1),X_est_R(:,2),'g-*','MarkerSize',12,'LineWidth',1.1);
plot(9,10.8,'rs','MarkerFaceColor','r','MarkerSize',12);
plot(9,0.75,'rs','MarkerFaceColor','r','MarkerSize',12);
plot(3,0.75,'rs','MarkerFaceColor','r','MarkerSize',12);
plot(3,10.8,'rs','MarkerFaceColor','r','MarkerSize',12);
axis([2 10 0 11.5]);
xlabel('东向距离/m'); 
ylabel('北向距离/m');
% legend('真实轨迹','双级EKF无NLOS检测','三角形检测算法','双级EKF和NLOS检测','基站');
legend('真实轨迹','双级EKF无NLOS检测','三角形检测算法','双级EKF和NLOS检测');
hold off;
title('轨迹图');

%%计算预测协方差矩阵
RMSE_I_U=sqrt((sum((X_est_R_I_U(:,1)-P_P(:,1)).^2+(X_est_R_I_U(:,2)-P_P(:,2)).^2))/41);
RMSE_I=sqrt((sum((X_est_R_I(:,1)-P_P(:,1)).^2+(X_est_R_I(:,2)-P_P(:,2)).^2))/41);
RMSE_R_I=sqrt((sum((X_est_R(:,1)-P_P(:,1)).^2+(X_est_R(:,2)-P_P(:,2)).^2))/41);





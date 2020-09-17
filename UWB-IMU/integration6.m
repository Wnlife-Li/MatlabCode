
clc;
clear all;
%%%%%%%%%%%%%%-------------基于UWB和IMU的室内定位融合算法--------------%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%===============【三角形算法   实验区域是小范围     噪声使用了awg函数】===============%%%%%%%%%%%%%%%%%%%%%%%
%----1.模拟惯性导航的数据
%%%---1.1 6个基站的坐标
x1=[1 1];
x2=[1 10];
x3=[5 10];
x4=[10 10];
x5=[10 1];
x6=[5 1];
bs=[x1;x2;x3;x4;x5;x6];
%%%---1.2 航向信息
h=zeros(41,1);
for i=1:10
    h(i)=0;
    h(i+10)=90;
    h(i+20)=180;
    h(i+30)=270;
end
h(41)=270;
% % 给航向角加入噪声
% figure;
% plot(h,'g');
% hold on;
h=awgn(h,-10);%给航向角加入高斯白噪声
% plot(h,'r');

%将航向变为弧度数
for i=1:41
   h(i)=(h(i)*pi)/180;
end
%%%--- 1.4 步数 
N_P=41;
%%%--- 1.5 步长
N_S=zeros(41,1);
for i=1:41
    N_S(i)=0.6;
end
% figure;
% plot(N_S,'r');
% hold on;
N_S=awgn(N_S,30);%给步长加入高斯白噪声
% plot(N_S,'g');


%%%--- 1.6 初试位置(2 2)


%模拟实际的行走坐标位置
P_P=zeros(41,2);
P_P(1,1)=2;
P_P(1,2)=2;
for i=1:10
  P_P(i+1,1)=2;
  P_P(i+1,2)=0.6+P_P(i,2);
end
for i=12:21
   P_P(i,2)=8;
   P_P(i,1)=P_P(i-1,1)+0.6;
end
for i=22:31
   P_P(i,1)=8; 
   P_P(i,2)=P_P(i-1,2)-0.6;
end
for i=32:41
   P_P(i,1)=P_P(i-1,1)-0.6;
   P_P(i,2)=2;
end
% figure;
% plot(P_P(:,1),P_P(:,2));
%%%模拟UWB的测量距离
d_UWB_1=zeros(41,1);
for i=1:41
   d_UWB_1(i)= sqrt((P_P(i,1)-x1(1))^2+(P_P(i,2)-x1(2))^2);
end
d_UWB_2=zeros(41,1);
for i=1:41
   d_UWB_2(i)= sqrt((P_P(i,1)-x2(1))^2+(P_P(i,2)-x2(2))^2);
end
d_UWB_3=zeros(41,1);
for i=1:41
   d_UWB_3(i)= sqrt((P_P(i,1)-x3(1))^2+(P_P(i,2)-x3(2))^2);
end
d_UWB_4=zeros(41,1);
for i=1:41
   d_UWB_4(i)= sqrt((P_P(i,1)-x4(1))^2+(P_P(i,2)-x4(2))^2);
end
d_UWB_5=zeros(41,1);
for i=1:41
   d_UWB_5(i)= sqrt((P_P(i,1)-x5(1))^2+(P_P(i,2)-x5(2))^2);
end
d_UWB_6=zeros(41,1);
for i=1:41
   d_UWB_6(i)= sqrt((P_P(i,1)-x6(1))^2+(P_P(i,2)-x6(2))^2);
end
%%测试UWB测量距离的大小
% figure;
% plot(d_UWB_1,'r');
% hold on;

%%%++++++++++++++++++++++++++++++给所有UWB测量数据加入高斯噪声
% Z_UWB_d_1=sqrt(0.0025).*randn(41,1);
% d_UWB_1=d_UWB_1+Z_UWB_d_1;
% Z_UWB_d_2=sqrt(0.0025).*randn(41,1);
% d_UWB_2=d_UWB_2+Z_UWB_d_2;
% Z_UWB_d_3=sqrt(0.0025).*randn(41,1);
% d_UWB_3=d_UWB_3+Z_UWB_d_3;
% Z_UWB_d_4=sqrt(0.0025).*randn(41,1);
% d_UWB_4=d_UWB_4+Z_UWB_d_4;
% Z_UWB_d_5=sqrt(0.0025).*randn(41,1);
% d_UWB_5=d_UWB_5+Z_UWB_d_5;
% Z_UWB_d_6=sqrt(0.0025).*randn(41,1);
% d_UWB_6=d_UWB_6+Z_UWB_d_6;
d_UWB_1=awgn(d_UWB_1,20);
d_UWB_2=awgn(d_UWB_2,20);
d_UWB_3=awgn(d_UWB_3,20);
d_UWB_4=awgn(d_UWB_4,20);
d_UWB_5=awgn(d_UWB_5,20);
d_UWB_6=awgn(d_UWB_6,20);

%测试加了噪声以后的UWB测量距离的大小
% plot(d_UWB_1,'g');

%++++++++++++++++++++++++++++++++给3和4基站随即加入正向偏差(正向偏差大小为2m)
%给3基站加入30个正向偏差
R_UWB_3=randperm(40)+1;
R_UWB_3_U=R_UWB_3(1:30);
% figure;
% plot(d_UWB_3,'ro');
% hold on;
for i=1:30
   d_UWB_3(R_UWB_3_U(i))= d_UWB_3(R_UWB_3_U(i))+rand(1)+1.2;
end
% plot(d_UWB_3,'go','MarkerSize',13);
%给4基站加入30个正向偏差
R_UWB_4=randperm(40)+1;
R_UWB_4_U=R_UWB_4(1:30);
% figure;
% plot(d_UWB_4,'ro');
% hold on;
for i=1:30
   d_UWB_4(R_UWB_4_U(i))= d_UWB_4(R_UWB_4_U(i))+rand(1)+1.2;
end
% plot(d_UWB_4,'go','MarkerSize',13);

% %给6基站加入30个正向偏差
% R_UWB_6=randperm(40)+1;
% R_UWB_6_U=R_UWB_6(1:30);
% for i=1:30
%    d_UWB_6(R_UWB_6_U(i))= d_UWB_6(R_UWB_6_U(i))+rand(1)+0.6;
% end
% 
% %给1基站加入30个正向偏差
% R_UWB_1=randperm(40)+1;
% R_UWB_1_U=R_UWB_1(1:30);
% for i=1:30
%    d_UWB_1(R_UWB_1_U(i))= d_UWB_1(R_UWB_1_U(i))+rand(1)+0.6;
% end




%%%%%---------------------------单独惯性导航算法----------------------%%%%%%%%
len=N_P;%仿真步数
X_est = zeros(len,4);%EKF估计值
X_est(1,:)=[2,2,0.6,0];
x_hat_P = [2,2,0.6,0]';
x_forecast = zeros(4,1);%预测值

Qk_P = 0.5*diag([0.5; 0.5; 0.1; (5*pi)/180])^2;%动态噪声矩阵
Rk_P = 0.01*diag([0.1; (5*pi)/180])^2;%观测噪声矩阵       
Z_P=[N_S,h];  %惯性导航的测量值（一个步长一个航向）
Pk_P = 1*eye(4);%测量协方差矩阵的初始值，%%%%%%%%%%%%%%%%%%%%%%%大小待定
Pkk_1_P = eye(4);%预测协方差矩阵

 % 观测矩阵,是个固定值
H_P=[0,0,1,0;
    0,0,0,1];
for k=2:len
    % 1 状态预测    
    ex = x_hat_P(1) + x_hat_P(3)*sin(x_hat_P(4));
    ny = x_hat_P(2) + x_hat_P(3)*cos(x_hat_P(4));
    s=x_hat_P(3);
    f=x_hat_P(4);
    x_forecast = [ex; ny; s; f];%预测值
    % 2  观测预测
    y_yuce=H_P*x_forecast;
    %  状态矩阵
    F = zeros(4,4);
    F(1,1) = 1; F(1,2) = 0; F(1,3) = sin(x_forecast(4)); F(1,4) = x_forecast(3)*cos(x_forecast(4));
    F(2,1) = 0; F(2,2) = 1; F(2,3)=cos(x_forecast(4)); F(2,4) = -x_forecast(3)*sin(x_forecast(4));
    F(3,1) = 0; F(3,2) = 0; F(3,3) = 1; F(3,4) = 0;
    F(4,1) = 0; F(4,2) = 0; F(4,3) = 0; F(4,4) = 1;
    Pkk_1_P = F*Pk_P*F'+Qk_P;%惯导的预测协方差矩阵   
%     计算卡尔曼增益
    Kk = Pkk_1_P*H_P'*(H_P*Pkk_1_P*H_P'+Rk_P)^-1; 
    %获取EKF估计值
    x_hat_P = x_forecast+Kk*(Z_P(k,:)'-y_yuce);%校正
    %计算状态向量的估计协方差矩阵
    Pk_P = (eye(4)-Kk*H_P)*Pkk_1_P;  
    %将滤波结果保存在矩阵中
    X_est(k,:) = x_hat_P';  
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据融合算法%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

              %%%%%%%%%%%%%%%%%%%%%%    【融合部分EKF数据的初始化】    %%%%%%%%%%%%%%%%%%%%
               
% %假设d为第n步标签到基站的距离  ---- 假设有6个基站
d=[d_UWB_1,d_UWB_2,d_UWB_3,d_UWB_4,d_UWB_5,d_UWB_6];
% residual=zeros(6,1);
Z_R=[d_UWB_1,d_UWB_2,d_UWB_3,d_UWB_4,d_UWB_5,d_UWB_6,N_S,h]; %  -----  融合EKF的测量值 （6个距离，一个步长和一个航向） n行8列
HH_R=zeros(8,4);
X_est_R = zeros(len,4);     %----  融合的EKF最优估计值
X_est_R(1,:)=[2,2,0.6,0];
P_R=1*eye(4);%测量协方差矩阵的初始值
% P_R(4,4)=P_R(4,4)*40;
R_F = diag([0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.1; (0.5*pi)/180])^2;  %【融合】观测噪声矩阵
x_R=[2,2,0.6,0]';%状态变量的初始值
R_R_R=zeros(len,6);
d_t_1=zeros(6,1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%       【惯性导航的EKF的初始化】    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%x_hat为状态向量，x_hat=[ex,yn,s,f],e为东向坐标，n为北向坐标，s为步长，f为航向
%东北位置坐标的噪声We-N(0,1^2),Wn-N(0,1^2),步长噪声Ws-N(0,0.1^2)(0.1m/s),航向的噪声为Wf-N(0,5^2)(5。)
%Z=[plength dir]为测量值，plength为步长，dir为航向角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 惯性导航使用ekf 得出当前位置
Qk = 0.5*diag([0.5; 0.5; 0.1; (5*pi)/180])^2;%动态噪声矩阵
Rk = 0.01*diag([0.1; (5*pi)/180])^2;%观测噪声矩阵       
Z=[N_S,h];  %惯性导航的测量值（一个步长一个航向）
Pk = 1*eye(4);%测量协方差矩阵的初始值，%%%%%%%%%%%%%%%%%%%%%%%大小待定
Pkk_1 = eye(4);%预测协方差矩阵
x_hat = [2,2,0.6,0]';%状态变量的初始值[ex,ny,s,f]%%%%%%%%%%%%%%%%%%%%%%%大小待定
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
    msd=sqrt((x_hat(1)-X_est_R(k-1,1))^2+(x_hat(2)-X_est_R(k-1,2))^2);
    %计算前一个时刻基站到移动节点的距离
    for g=1:6
        d_t_1(g)=sqrt((X_est_R(k-1,1)-bs(g,1))^2+(X_est_R(k-1,2)-bs(g,2))^2);%%---????上一个时刻的位置应该用融合算出来的位置
    end
    %计算残差
    residual=ones(6,1);
    for g=1:6                          %-----此处假设有6个基站
%       residual(g)=abs(msd-abs(d(k-1,g)-d(k,g))-Threshold)*k_r;   %Threshold=-1待定  ------   k_r为残差到构造观测矩阵的一个调节系数   
        R_R_R(k,g)= msd-abs(d_t_1(g)-d(k,g));  
        if(R_R_R(k,g)<-0.1) 
            residual(g)=abs( R_R_R(k,g))*1000000;
        end
    end
    %计算融合时需要的测量噪声矩阵
    de=diag([residual(1); residual(2); residual(3); residual(4); residual(5); residual(6); 1; 1]);%融合噪声的矩阵的残差系数
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
    
    HH_R(5,1)=(x_forecast_R(1)- x5(1))*((x_forecast_R(1)-  x5(1))^2 +(x_forecast_R(1)- x5(2))^2)^(-0.5);
    HH_R(5,2)=(x_forecast_R(2)- x5(2))*((x_forecast_R(1)-  x5(1))^2 +(x_forecast_R(1)- x5(2))^2)^(-0.5);
    
    HH_R(6,1)=(x_forecast_R(1)- x6(1))*((x_forecast_R(1)-  x6(1))^2 +(x_forecast_R(1)- x6(2))^2)^(-0.5);
    HH_R(6,2)=(x_forecast_R(2)- x6(2))*((x_forecast_R(1)-  x6(1))^2 +(x_forecast_R(1)- x6(2))^2)^(-0.5); 
   
    HH_R(7,3)=1; 
    HH_R(8,4)=1; 
    
    %预测值到测量值
    y_R(1,1)=sqrt((x1(1)-  x_forecast_R(1))^2 +(x1(2)-x_forecast_R(2))^2);
    y_R(2,1)=sqrt((x_forecast_R(1)-  x2(1))^2 +(x_forecast_R(2)- x2(2))^2);
    y_R(3,1)=sqrt((x_forecast_R(1)-  x3(1))^2 +(x_forecast_R(2)- x3(2))^2);
    y_R(4,1)=sqrt((x_forecast_R(1)-  x4(1))^2 +(x_forecast_R(2)- x4(2))^2);
    y_R(5,1)=sqrt((x_forecast_R(1)-  x5(1))^2 +(x_forecast_R(2)- x5(2))^2);
    y_R(6,1)=sqrt((x_forecast_R(1)-  x6(1))^2 +(x_forecast_R(2)- x6(2))^2);
    y_R(7,1)=x_forecast_R(3);
    y_R(8,1)=x_forecast_R(4);
    
    %计算卡尔曼增益
    K_R = Pkk_R*HH_R'*(HH_R*Pkk_R*HH_R'+R_E)^-1; 
    %最优估计值
    x_R = x_forecast_R+K_R*(Z_R(k,:)'-y_R);%校正
    %计算状态向量的估计协方差矩阵
    P_R = (eye(4)-K_R*HH_R)*Pkk_R;  
    %将滤波结果保存在矩阵中，融合后的最优估计值
    X_est_R(k,:) = x_R';  
    
end
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


% %----描述行走过程中的真实轨迹
tx=2;
ty=2;
l=6;
w=6;
x=[tx,tx+l,tx+l,tx,tx];
y=[ty,ty,ty+w,ty+w,ty];
figure;
plot(x,y,'k','LineWidth',1.5);
hold on
%%%惯性导航的轨迹
plot(X_est(:,1),X_est(:,2),'g-*','MarkerSize',8);
%%%融合的轨迹
plot(X_est_R(:,1),X_est_R(:,2),'m-x','MarkerSize',10);
plot(1,1,'bs','MarkerFaceColor',[0 0 1],'MarkerSize',12);
plot(1,10,'bs','MarkerFaceColor',[0 0 1],'MarkerSize',12);
plot(5,10,'bs','MarkerFaceColor',[0 0 1],'MarkerSize',12);
plot(10,10,'bs','MarkerFaceColor',[0 0 1],'MarkerSize',12);
plot(10,1,'bs','MarkerFaceColor',[0 0 1],'MarkerSize',12);
plot(5,1,'bs','MarkerFaceColor',[0 0 1],'MarkerSize',12);
axis([0 11 0 11]);
xlabel('东向距离/m'); 
ylabel('北向距离/m');
legend('真实轨迹','PDR','UWB+PDR','基站');
hold off;


figure;
RMSE_I_U=sqrt((sum((X_est(:,1)-P_P(:,1)).^2+(X_est(:,2)-P_P(:,2)).^2))/41);
RMSE_I_U_R=sqrt((sum((X_est_R(:,1)-P_P(:,1)).^2+(X_est_R(:,2)-P_P(:,2)).^2))/41);
RMSE=[RMSE_I_U RMSE_I_U_R];
bar(RMSE,'BarWidth',0.2);

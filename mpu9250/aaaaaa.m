%%%%%%%%%%%%%%%%%%%%%
%测试pdr程序  V:2.0--[放到肩膀上的]
%input：3轴加速度和航向角度
%output：步长，步数，和一步的航向
%改进点：相对于v1加入了iHDE算法
%p2.5的数据很好
%专门测 【矩形】
%%%%%%%%%%%%%%%%%%%%%
clc;
%1.导入数据
newData = importdata('./P2/3.tsv', '\t', 2);
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
asup_h=asum(50:680);
% g=1.02649;
g=1.0704;
%apsum减去重力以后的加速度
apsum=asup_h-g;
figure; 
plot(apsum);
title('三轴加速度的和去除重力以后的加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2'); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%加速度处理
[N_a,M_a]=size(apsum);
%卡尔曼滤波处理加速度
Q_a = 0.1;%系统噪声
R_a = 0.3;% 测量噪声

% 分配空间
xhat_a=zeros(N_a,1);       % x的后验估计
P_a=0.5;           % 后验方差估计  n*n
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

for i=2:Z-8
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
    if pflag==1&&pclimbing>5&&apsum_a2(tflag+pclimbing)>0.035
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on;
scatter(sflag,apsum_a2(sflag),90,'rp');
scatter(pb,apsum_a2(pb),70,'gO');
plot(apsum_a2,'b');
% title('加速度的步伐和零点检测');
xlabel('采样点');
ylabel('加速度幅值(m/s2)');
legend('峰值点','零点','加速度幅值');
pzero
step





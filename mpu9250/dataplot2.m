%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%航向角预处理程序%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
newData = importdata('./G/G6.tsv', '\t', 2);
data=newData.data;
figure;
% subplot(2,1,1);
% plot(data(:,7));
% xlabel('时间/s');
% ylabel('-z轴角度/°');
% title('航向角角曲线');
% subplot(2,1,2);
[m, n]=size(data);
dir_p=[];
 for i=1:m-14
     tdirp=sum(data(i:i+14,7))/15;  
     dir_p = cat(1,dir_p,tdirp);
 end
 
subplot(2,1,1);
plot(-data(:,7));
xlabel('时间/s');
ylabel('+z轴角度/°');
title('航向角曲线');
subplot(2,1,2);
plot(-dir_p);
xlabel('时间/s');
ylabel('+z轴角度/°');
title('平滑滤波后的航向角曲线');

%iHDE算法反馈航向角
SLP=0;%直行标记
[N,M]=size(dir_p);
dir_arr=zeros(1,4);
error=zeros(N,1);%角度误差,大于0往左偏，小于0往右偏。航向角减去误差值
for k=4:N
    %直行判断
    for t=1:3
        dir_arr(t)=abs(dir_p(k-t)-mean(dir_p(k-3:k)));
    end
    max_dir=max(dir_arr);
    if(max_dir<15) 
        SLP=1;
    end
   
    if SLP==1
      %直行情况下，航向角的观测值值等于三步内航向角的平均值
      %航向角的误差值等于当前航向角的值减去航向角的观测值
      error(k)=dir_p(k)-mean(dir_p(k-3:k));  
    end
    
   %主方向判断
   E=45-mod(dir_p(k),90);
    if abs(E)>30&&SLP==1%说明是主方向上直行,取观测值为主方向的值，将原先的误差值覆盖
        if dir_p(k)>0&&dir_p(k)<15
            error(k)=dir_p(k)-0;
        end
        if dir_p(k)>345&&dir_p(k)<360
            error(k)=dir_p(k)-360;
        end
        if dir_p(k)>75&&dir_p(k)<105
            error(k)=dir_p(k)-90;
        end
         if dir_p(k)>165&&dir_p(k)<195
            error(k)=dir_p(k)-180;
        end
        if dir_p(k)>255&&dir_p(k)<285
            error(k)=dir_p(k)-90;
        end               
    end     
end

%将航向误差值输入到卡尔曼滤波中

dir_Q = 5;%系统噪声
dir_R = 5;% 测量噪声

dir_xhat=zeros(N,1);       % x的后验估计
dir_P=5;           % 后验方差估计  n*n
dir_xhatminus=zeros(N,1);  % x的先验估计
dir_Pminus=0;      % n*n
dir_K=0;   % Kalman增益  n*m

dir_cor=zeros(N,1);
% 估计的初始值都为默认的0，即P=[0 0;0 0],xhat=0
for r = 2:N         
    % 时间更新过程
    dir_xhatminus(r) = dir_xhat(r-1);
    dir_Pminus= dir_P+dir_Q;
    
    % 测量更新过程
    dir_K = dir_Pminus*inv( dir_Pminus+dir_R );
    dir_xhat(k) = dir_xhatminus(k)+dir_K*(error(k)-dir_xhatminus(k));
    dir_P = (1-dir_K)*dir_Pminus;
    
    %航向纠正，航向值减去误差值
    dir_cor=dir_p(k)-dir_xhat(k);
end

figure; 
plot(dir_cor);
title('iHDE算法处理后的航向角');
xlabel('单位：采样点');
ylabel('单位：m/s2'); 


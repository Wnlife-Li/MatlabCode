%加速度测试预处理程序
%测试pdr程序 [放在脚上的]

newData = importdata('./F4/5.tsv', '\t', 2);
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
apsum_e1=[];
 for i=1:m-10
     temp1=sum(apsum(i:i+10,:))/11;  
     apsum_e1 = cat(1,apsum_e1,temp1);
 end
 
 [M,N]=size(apsum_e1);
 apsum_e2=[];
 for i=1:M-4
     temp2=sum(apsum_e1(i:i+4,:))/5;  
     apsum_e2 = cat(1,apsum_e2,temp2);
 end
 
 
%  航向平滑滤波
 dir_p=[];
 for i=1:m-14
     tdirp=sum(data(i:i+14,10))/15;  
     dir_p = cat(1,dir_p,tdirp);
 end

%平滑滤波后的行和列数
[z,q]=size(apsum_e2);

step=0;%步数
sflag=0;%步伐峰值标记位
status=1;%当前状态，1为行走状态，2为峰值状态
i0=10;

for i=20:z-20
    
    if apsum_e2(i)>0.05
        status=1;
    end
%误判判断
    if status==2&&i-i0<10
        if apsum_e2(i)<apsum_e2(sflag(step+1))
            sflag(step+1)=i;           
        end
    end 

%stance检测
    if apsum_e2(i)<apsum_e2(i+1)&&apsum_e2(i)<apsum_e2(i-1)
%         if apsum_e(i)>(-0.1)&&apsum_e(i)<0&&(apsum_e(i-15)>0.1||apsum_e(i+20)>0.1)&&i-i0>20       
        if apsum_e2(i)<0.06&&(apsum_e2(i-15)>0.15||apsum_e2(i+15)>0.15)&&i-i0>20   
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
scatter(sflag,apsum_e2(sflag));
plot(apsum_e2);
title('平滑滤波后的和加速度');
xlabel('单位：采样点');
ylabel('单位：m/s2');
legend('步伐判定临界点')
step

%%原始椭球拟合算法
%生成随机椭球球面数据
close all;
clc;
A = 300;     % x方向上的轴半径
B = 300;     % y方向上的轴半径
C = 300;     % z方向上的轴半径
x0 = -120;   %椭球球心x坐标
y0 = -67;    %椭球球心y坐标
z0 = 406;    %椭球球心z坐标
SNR = 30;    %信噪比
 
num_alfa = 100;
num_sita = 50;
alfa = (0:num_alfa-1)*1*pi/num_alfa;
sita = (0:num_sita-1)*2*pi/num_sita;
x = zeros(num_alfa,num_sita);
y = zeros(num_alfa,num_sita);
z = zeros(num_alfa,num_sita);
for i = 1:num_alfa
    for j = 1:num_sita
        x(i,j) = x0 + A*sin(alfa(i))*cos(sita(j));
        y(i,j) = y0 + B*sin(alfa(i))*sin(sita(j));
        z(i,j) = z0 + C*cos(alfa(i));
    end
end
 
x = reshape(x,num_alfa*num_sita,1);    %转换成一维的数组便于后续的处理
y = reshape(y,num_alfa*num_sita,1);
z = reshape(z,num_alfa*num_sita,1);
figure;
plot3(x,y,z,'*');
title('生成的没有噪声的椭球面数据');
 
%加入均值为0的高斯分布噪声 
amp = 10^(-SNR/20)*A;
x = x + amp*rand(num_alfa*num_sita,1);
y = y + amp*B/A*rand(num_alfa*num_sita,1);
z = z + amp*C/A*rand(num_alfa*num_sita,1);
figure;
plot3(x,y,z,'*');
string = ['加入噪声的椭球面数据，SNR=',num2str(SNR),'dB'];
title(string);
%% 空间二次曲面拟合算法
num_points = length(x);
%一次项统计平均
x_avr = sum(x)/num_points;
y_avr = sum(y)/num_points;
z_avr = sum(z)/num_points;
%二次项统计平均
xx_avr = sum(x.*x)/num_points;
yy_avr = sum(y.*y)/num_points;
zz_avr = sum(z.*z)/num_points;
xy_avr = sum(x.*y)/num_points;
xz_avr = sum(x.*z)/num_points;
yz_avr = sum(y.*z)/num_points;
%三次项统计平均
xxx_avr = sum(x.*x.*x)/num_points;
xxy_avr = sum(x.*x.*y)/num_points;
xxz_avr = sum(x.*x.*z)/num_points;
xyy_avr = sum(x.*y.*y)/num_points;
xzz_avr = sum(x.*z.*z)/num_points;
yyy_avr = sum(y.*y.*y)/num_points;
yyz_avr = sum(y.*y.*z)/num_points;
yzz_avr = sum(y.*z.*z)/num_points;
zzz_avr = sum(z.*z.*z)/num_points;
%四次项统计平均
yyyy_avr = sum(y.*y.*y.*y)/num_points;
zzzz_avr = sum(z.*z.*z.*z)/num_points;
xxyy_avr = sum(x.*x.*y.*y)/num_points;
xxzz_avr = sum(x.*x.*z.*z)/num_points;
yyzz_avr = sum(y.*y.*z.*z)/num_points;
 
%计算求解线性方程的系数矩阵
A0 = [yyyy_avr yyzz_avr xyy_avr yyy_avr yyz_avr yy_avr;
     yyzz_avr zzzz_avr xzz_avr yzz_avr zzz_avr zz_avr;
     xyy_avr  xzz_avr  xx_avr  xy_avr  xz_avr  x_avr;
     yyy_avr  yzz_avr  xy_avr  yy_avr  yz_avr  y_avr;
     yyz_avr  zzz_avr  xz_avr  yz_avr  zz_avr  z_avr;
     yy_avr   zz_avr   x_avr   y_avr   z_avr   1;];
%计算非齐次项
b = [-xxyy_avr;
     -xxzz_avr;
     -xxx_avr;
     -xxy_avr;
     -xxz_avr;
     -xx_avr];
 
resoult = inv(A0)*b;
%resoult = solution_equations_n_yuan(A0,b);
 
x00 = -resoult(3)/2;                  %拟合出的x坐标
y00 = -resoult(4)/(2*resoult(1));     %拟合出的y坐标
z00 = -resoult(5)/(2*resoult(2));     %拟合出的z坐标
 
AA = sqrt(x00*x00 + resoult(1)*y00*y00 + resoult(2)*z00*z00 - resoult(6));   % 拟合出的x方向上的轴半径
BB = AA/sqrt(resoult(1));                                                    % 拟合出的y方向上的轴半径
CC = AA/sqrt(resoult(2));                                                    % 拟合出的z方向上的轴半径
 
x=x-x00;
y=y-y00;
z=z-z00;

xmin = min(x);
ymin = min(y);
zmin = min(z);
xmax = max(x);
ymax = max(y);
zmax = max(z);

xc = 0.5*(xmax+xmin);
yc = 0.5*(ymax+ymin);
zc = 0.5*(zmax+zmin);

y=y*(AA/BB);
z=z*(AA/CC);

%拟合为球
figure;
plot3(x,y,z,'*');
title('球面数据');

fprintf('拟合结果\n');
fprintf('a = %f, b = %f, c = %f, d = %f, e = %f, f = %f\n',resoult);
fprintf('x0 = %f, 相对误差%f\n',x00,abs((x00-x0)/x0));
fprintf('y0 = %f, 相对误差%f\n',y00,abs((y00-y0)/y0));
fprintf('z0 = %f, 相对误差%f\n',z00,abs((z00-z0)/z0));
fprintf('A = %f,  相对误差%f\n',AA,abs((A-AA)/A));
fprintf('B = %f,  相对误差%f\n',BB,abs((B-BB)/B));
fprintf('C = %f,  相对误差%f\n',CC,abs((C-CC)/C));


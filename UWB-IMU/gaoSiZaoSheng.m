%%%%%%产生随机数

% clear,clc;
% N=0:1000;
% fs=1024;
% t=N./fs; 
% y=3*sin(2*pi*t);
% x=wgn(1,1001,2);
% i=y+x;
% % i=awgn(y,2);
% subplot(3,1,1),plot(x);
% subplot(3,1,2),plot(y);
% subplot(3,1,3),plot(i);

A=randperm(10);
A
A(1:5)

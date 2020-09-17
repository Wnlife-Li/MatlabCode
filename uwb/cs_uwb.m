%1-D信号压缩传感的实现（正交匹配追踪法OMP）
%时域测试信号生成
K=15;              %稀疏度（做FFT可以看出来）
N=500;            %信号长度
M=100;             %测量数(M>=K*log(N/K))
Tp = 1e-8;           %脉冲时间宽度
Nc = 8;           %正弦波周期个数
A = 1;               %脉冲幅度
f = Nc / Tp;            %频率
x = sin(2.*pi.*f.*linspace(0,Tp,N));   %UWB信号
Phi=randn(M,N);                      %观测矩阵
s=Phi*x.';                           
m=2*K;
Psi=fft(eye(N,N))/sqrt(N);
T=Phi*Psi';
hat_y=zeros(1,N);
Aug_t=[];
r_n=s;
for times=1:m
    for col=1:N
        product(col)=abs(T(:,col)'*r_n);
    end
    [val,pos]=max(product);
    Aug_t=[Aug_t,T(:,pos)];
    T(:,pos)=zeros(M,1);
    aug_y=(Aug_t'*Aug_t)^(-1)*Aug_t'*s;
    r_n=s-Aug_t*aug_y;
    pos_array(times)=pos;            
end
hat_y(pos_array)=aug_y;             %重构的谱域向量
hat_x=real(Psi'*hat_y.');          %做逆傅里叶变换重构得到的时域信号
subplot(211)
plot(hat_x,'k.-')                 %重建信号
ylim([-1 1]);
title('恢复出的信号x');
subplot(212)
plot(x,'r')                      %原始信号
title('原信号x');
norm(hat_x.'-x)/norm(x)             %重构误差
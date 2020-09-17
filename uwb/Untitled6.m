%1-D信号压缩传感的实现（正交匹配追踪法OMP）
%时域测试信号生成
K=7;              %稀疏度（做FFT可以看出来）
N=256;            %信号长度
M=64;  

f1=50;
f2=100;
f3=200;
f4=400;
fs=800;
ts=1/fs;
Ts=1:N;
x=0.3*cos(2*pi*f1*Ts*ts)+0.6*cos(2*pi*f2*Ts*ts)+0.1*cos(2*pi*f3*Ts*ts)+0.9*cos(2*pi*f4*Ts*ts);





Phi=randn(M,N);
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
hat_y(pos_array)=aug_y;
hat_x=real(Psi'*hat_y.');
subplot(211)
plot(hat_x,'k.-')
subplot(212)
plot(x,'r')
norm(hat_x.'-x)/norm(x)
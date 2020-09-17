
%%%--- 1.5 ²½³¤
N_S=zeros(41,1);
for i=1:41
    N_S(i)=0.6;
end
figure;
subplot(2,1,1);
plot(N_S);
hold on;
% N_S=N_S+sqrt(0.005).*randn(41,1);
N_S=awgn(N_S,50);
subplot(2,1,2);
plot(N_S);
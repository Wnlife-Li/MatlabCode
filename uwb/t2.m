fc=10e10;
Tm=0.9e-9;
taul=0.2e-9;
tau2=0.3e-9;
tau3=0.4e-9;
[a1]= waveform(fc,Tm,taul);
[a2]= waveform(fc,Tm,tau2);
[a3]= waveform(fc,Tm,tau3);
figure(1);
time=linspace(-Tm/2,Tm/2,length(a1));
p=plot(time,a1,time,a2,time,a3)
X=xlabel('Time [s]');
set(X,'FontSize',14);
Y=ylabel('Amplitude [V]');
set(Y,'FontSize',14)
grid on;

function [w]= waveform(fc,Tm,tau);
dt = 1 / fc;
OVER = floor(Tm/dt);
e = mod(OVER,2);
kbk = floor(OVER/2);
tmp = linspace(dt,Tm/2,kbk);
s = (1-4.*pi.*((tmp./tau).^2)).* ...
exp(-2.*pi.*((tmp./tau).^2));
if e
for k=1:length(s)
y(kbk+1)=1;
y(kbk+1+k)=s(k);
y(kbk+1-k)=s(k);
end
else
for k=1:length(s)
y(kbk+k)=s(k);
y(kbk+1-k)=s(k);
end
end
E = sum((y.^2).*dt);
w = y ./ (E^0.5);
end
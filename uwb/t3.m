Pow = -30;
fc = 50e9;
numbits = 2;
Ts = 3e-9;
Ns = 5;
Tc = 1e-9;
Nh = 3;
Np = 5;
Tm = 0.5e-9;
tau = 0.25e-9;
dPPM = 0.5e-9;
G = 1;
bits = bit(numbits);
repbits = repcode(bits,Ns);
THcode =TH(Nh,Np);
[PPMTHseq,THseq] = PPM_TH(repbits,fc,Tc,Ts,dPPM,THcode);
power = (10^(Pow/10))/1000;
Ex = power * Ts;
w0 = waveform(fc,Tm,tau);
wtx = w0 .* sqrt(Ex);
Sa = conv(PPMTHseq,wtx);
Sb = conv(THseq,wtx);
L = (floor(Ts*fc))*Ns*numbits;
Stx = Sa(1:L);
ref = Sb(1:L);
if G
F = figure(1);
set(F,'Position',[32 223 951 420]);
tmax = numbits*Ns*Ts;
time = linspace(0,tmax,length(Stx));
P = plot(time,Stx);
set(P,'LineWidth',[2]);
ylow=-1.5*abs(min(wtx));
yhigh=1.5*max(wtx);
axis([0 tmax ylow yhigh]);
AX=gca;
set(AX,'FontSize',12);
X=xlabel('Time [s]');
set(X,'FontSize',14);
Y=ylabel('Amplitude [V]');
set(Y,'FontSize',14);
for j = 1 : numbits
tj = (j-1)*Ns*Ts;
L1=line([tj tj],[ylow yhigh]);
set(L1,'Color',[0 0 0],'LineStyle', '--','LineWidth',[2]);
for k = 0 : Ns-1
if k > 0
tn = tj + k*Nh*Tc;
L2=line([tn tn],[ylow yhigh]);
set(L2,'Color',[0.5 0.5 0.5],'LineStyle', '-.','LineWidth',[2]);
end
for q = 1 : Nh-1
th = tj + k*Nh*Tc + q*Tc;
L3=line([th th],[0.8*ylow 0.8*yhigh]);
set(L3,'Color',[0 0 0],'LineStyle', ':','LineWidth',[1]);
end
end
end
end

%子函数 bits
function [bits]= bit(numbits)
bits=rand(1,numbits)>0.5;
end
%子函数 repcode
function [repbits]= repcode(bits,Ns)
numbits = length(bits);
temprect=ones(1,Ns);
temp1=zeros(1,numbits*Ns);
temp1(1:Ns:1+Ns*(numbits-1))=bits;
temp2=conv(temp1,temprect);
repbits=temp2(1:Ns*numbits);
end
%子函数 TH
function [THcode]=TH(Nh,Np);
THcode = floor(rand(1,Np).*Nh);
end
%子函数 PPM_TH
function [PPMTHseq,THseq] = PPM_TH(seq,fc,Tc,Ts,dPPM,THcode)
dt = 1 ./ fc;
framesamples = floor(Ts./dt);
chipsamples = floor (Tc./dt);
PPMsamples = floor (dPPM./dt);
THp = length(THcode);
totlength = framesamples*length(seq);
PPMTHseq=zeros(1,totlength);
THseq=zeros(1,totlength);
for k = 1 : length(seq)
index = 1 + (k-1)*framesamples;
kTH =THcode(1+mod(k-1,THp));
index = index + kTH*chipsamples;
THseq(index) = 1;
index = index + PPMsamples*seq(k);
PPMTHseq(index) = 1;
end
end
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
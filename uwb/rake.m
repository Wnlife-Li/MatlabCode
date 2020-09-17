[bits,THcode,Stx,ref]= transmitter_2PPM_TH; 
tx=1;
c0=10^(-47/20);
d=2;
gamma=1.7;
[rx,ag]= pathloss(tx,c0,d,gamma);
TMG=ag^2;
fc=50e9;
[h0,hf,OT,ts,X]= UWBC1(fc,TMG);
SRX0=Stx.*ag;
numpulses=30000;
exno=[0 2 4 6];
[output,noise]= Gnoise2(SRX0,exno,numpulses); SRX=conv(Stx,hf);
SRX=SRX(1:length(Stx));
RX(1,:)=SRX+noise(1,:);
RX(2,:)=SRX+noise(2,:);
RX(3,:)=SRX+noise(3,:);
RX(4,:)=SRX+noise(4,:);
L=10;
S=10;
[G,T,NF,rec_A,rec_B,rec_D]= rakeselector(hf,fc,ts,L,S);
L=2;
S=2;
[G,T,NF,rec_A,rec_C,rec_E]= rakeselector(hf,fc,ts,L,S); dPPM=0.5e-9;
[mask_A]= PPMcorrmask_R(ref,fc,numpulses,dPPM,rec_A);
[mask_B]= PPMcorrmask_R(ref,fc,numpulses,dPPM,rec_B);
[mask_C]= PPMcorrmask_R(ref,fc,numpulses,dPPM,rec_C); [mask_D]= PPMcorrmask_R(ref,fc,numpulses,dPPM,rec_D); [mask_E]= PPMcorrmask_R(ref,fc,numpulses,dPPM,rec_E); numbit=10000;
Ns=3;
Ts=3e-9;
[RXbits,ABER]= PPMreceiver(RX,mask_A,fc,bits,numbit,Ns,Ts);
[RXbits,BBER]= PPMreceiver(RX,mask_B,fc,bits,numbit,Ns,Ts);
[RXbits,CBER]= PPMreceiver(RX,mask_C,fc,bits,numbit,Ns,Ts);
[RXbits,DBER]= PPMreceiver(RX,mask_D,fc,bits,numbit,Ns,Ts);
[RXbits,EBER]= PPMreceiver(RX,mask_E,fc,bits,numbit,Ns,Ts);
semilogy(exno,ABER,'-o',exno,DBER,'-s','linewidth',1.5);
legend('PRake','SRake');
axis([0 6 10^(-2) 1]);
grid on;

%子函数：transmitter_2PPM_TH
function [bits,THcode,Stx,ref]=transmitter_2PPM_TH
power = -30;
fc = 50e9;
numbits = 10000;
Ts = 3e-9;
Ns = 3;
Tc = 1e-9;
Nh = 3;
Np = 30000;
Tm = 0.5e-9;
tau = 0.25e-9;
dPPM = 0.5e-9;
G = 0;
bits = bit(numbits);%上文已给出改子程序
repbits = repcode(bits,Ns); %上文已给出改子程序
THcode = TH(Nh,Np); %上文已给出改子程序
[PPMTHseq,THseq] = PPM_TH(repbits,fc,Tc,Ts,dPPM,THcode); %上文已给出改子程序 power = (10^(Pow/10))/1000;
Ex = power * Ts;
w0 = waveform(fc,Tm,tau); %上文已给出改子程序
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
set(L1,'Color',[0 0 0],'LineStyle', ...
'--','LineWidth',[2]);
for k = 0 : Ns-1
if k > 0
tn = tj + k*Nh*Tc;
L2=line([tn tn],[ylow yhigh]);
set(L2,'Color',[0.5 0.5 0.5],'LineStyle', ...
'-.','LineWidth',[2]); end
for q = 1 : Nh-1
th = tj + k*Nh*Tc + q*Tc;
L3=line([th th],[0.8*ylow 0.8*yhigh]);
set(L3,'Color',[0 0 0],'LineStyle', ...
':','LineWidth',[1]); end
end
end
end
end

%子函数：pathloss
function [rx,attn] = pathloss(tx,c0,d,gamma)
attn = (c0/sqrt(d^gamma));
rx = attn .* tx;
end

%子函数：UWBC1
function [h0,hf,OT,ts,X] = UWBC1 (fc,TMG);
OT = 200e-9;
ts = 1e-9;
LAMBDA = 0.0223*1e9;
lambda = 2.5e9;
GAMMA = 7.1e-9;
gamma = 4.3e-9;
sigma1 = 10^(3.3941/10);
sigma2 = 10^(3.3941/10);
sigmax = 10^(3/10);
rdt = 0.001;
PT = 50;
G = 1;
dt = 1 / fc;
T = 1 / LAMBDA;
t = 1 / lambda;
i = 1;
CAT(i)=0;
next = 0;
while next < OT
i = i + 1;
next = next + expinv(rand,T);
if next < OT
CAT(i)= next;
end
end
NC = length(CAT);
logvar = (1/20)*((sigma1^2)+(sigma2^2))*log(10);
omega = 1;
pc = 0;
for i = 1 : NC
pc = pc + 1;
CT = CAT(i);
HT(pc) = CT;
next = 0;
mx = 10*log(omega)-(10*CT/GAMMA);
mu = (mx/log(10))-logvar;
a = 10^((mu+(sigma1*randn)+(sigma2*randn))/20);
HA(pc) = ((rand>0.5)*2-1).*a;
ccoeff = sigma1*randn;
while exp(-next/gamma)>rdt
pc = pc + 1;
next = next + expinv(rand,t);
HT(pc) = CT + next;
mx = 10*log(omega)-(10*CT/GAMMA)-(10*next/GAMMA); mu = (mx/log(10))-logvar;
a = 10^((mu+ccoeff+(sigma2*randn))/20);
HA(pc) = ((rand>0.5)*2-1).*a;
end
end
peak = abs(max(HA));
limit = peak/10^(PT/10);
HA = HA .* (abs(HA)>(limit.*ones(1,length(HA))));
for i = 1 : pc
itk = floor(HT(i)/dt);
h(itk+1) = HA(i);
end
N = floor(ts/dt);
L = N*ceil(length(h)/N);
h0 = zeros(1,L);
hf = h0;
h0(1:length(h)) = h;
for i = 1 : (length(h0)/N)
tmp = 0;
for j = 1 : N
tmp = tmp + h0(j+(i-1)*N);
end
hf(1+(i-1)*N) = tmp;
end
E_tot=sum(h.^2);
h0 = h0 / sqrt(E_tot);
E_tot=sum(hf.^2);
hf = hf / sqrt(E_tot);
mux = ((10*log(TMG))/log(10)) - (((sigmax^2)*log(10))/20); X = 10^((mux+(sigmax*randn))/20);
h0 = X.*h0;
hf = X.*hf;
if G
Tmax = dt*length(h0);
time = (0:dt:Tmax-dt);
figure(1)
S1=stem(time,h0);
AX=gca;
set(AX,'FontSize',12);
T=title('Continuous-Time Channel Impulse Response');
set(T,'FontSize',12);
x=xlabel('Time [s]');
set(x,'FontSize',12);
y=ylabel('Amplitude Gain');
set(y,'FontSize',12);
figure(2)
S2=stairs(time,hf);
AX=gca;
set(AX,'FontSize',12);
T=title('Discrete-Time Channel Impulse Response');
set(T,'FontSize',12);
x=xlabel('Time [s]');
set(x,'FontSize',12);
y=ylabel('Amplitude Gain');
set(y,'FontSize',12);
end
end

%子函数：Gnoise2
function [output,noise] =Gnoise2(input,exno,numpulses)
Ex = (1/numpulses)*sum(input.^2);
ExNo = 10.^(exno./10);
No = Ex ./ ExNo;
nstdv = sqrt(No./2);
for j = 1 : length(ExNo)
noise(j,:) = nstdv(j) .* randn(1,length(input)); output(j,:) = noise(j,:) + input;
end
end


function [G,T,NF,Arake,Srake,Prake] =rakeselector(hf,fc,ts,L,S) 
dt = 1 / fc;
ahf = abs(hf);
[s_val,s_ind] = sort(ahf);
NF = 0;
i = length(s_ind);
j = 0;
while (s_val(i)>0)&(i>0)
NF = NF + 1;
j = j + 1;
index = s_ind(i);
I(j) = index;
T(j) = (index-1)*dt;
G(j) = hf(index);
i = i - 1;
end
binsamples = floor(ts/dt);
if S > NF
S = NF;
end
if L > NF
L = NF;
end
Arake = zeros(1,NF*binsamples);
Srake = zeros(1,NF*binsamples);
Prake = zeros(1,NF*binsamples);
for nf = 1 : NF
x = I(nf);
y = G(nf);
Arake(x) = y;
if nf <= S
Srake(x) = y;
end
end % for nf = 1 : NF
[tv,ti] = sort(T);
TV = tv(1:L);
TI = ti(1:L);
tc = 0;
for nl = 1 : length(TV)
index = TI(nl);
x = I(index);
y = G(index);
Prake(x) = y;
tc = tc + 1;
L = L - 1;
end
end

%子函数：PPMcorrmask_R
function [mask] = PPMcorrmask_R(ref,fc,numpulses,dPPM,rake)
dt = 1 / fc;
LR = length(ref);
Epulse = (sum((ref.^2).*dt))/numpulses;
nref = ref./sqrt(Epulse);
mref = conv(nref,rake);
mref = mref(1:LR);
PPMsamples = floor (dPPM ./ dt);
sref(1:PPMsamples)=mref(LR-PPMsamples+1:LR);
sref(PPMsamples+1:LR)=mref(1:LR-PPMsamples);
mask = mref-sref;
end

%子函数：PPMreceiver
function [RXbits,BER] = PPMreceiver(R,mask,fc,bits,numbit,Ns,Ts)
HDSD = 1;
[N,L] = size(R);
RXbits = zeros(N,numbit);
dt = 1 / fc;
framesamples = floor(Ts ./ dt);
bitsamples = framesamples * Ns;
for n = 1 : N
rx = R(n,:);
mx = rx .* mask;
if HDSD == 1
for nb = 1 : numbit
mxk = mx(1+(nb-1)*bitsamples:bitsamples+(nb-1)*bitsamples);
No0 = 0;
No1 = 0;
for np = 1 : Ns
mxkp = mxk(1+(np-1)*framesamples:...
framesamples+(np-1)*framesamples); zp = sum(mxkp.*dt);
if zp > 0
No0 = No0 + 1; else
No1 = No1 + 1;
end
end
if No0 > No1
RXbits(n,nb) = 0;
else
RXbits(n,nb) = 1;
end
end
end
if HDSD == 2
for nb = 1 : numbit
mxk = mx(1+(nb-1)*bitsamples:bitsamples+ (nb-1)*bitsamples); zb = sum(mxk.*dt);
if zb > 0
RXbits(n,nb) = 0;
else
RXbits(n,nb) = 1;
end
end
end
end
for n = 1 : N
WB = sum(abs(bits-RXbits(n,:)));
BER(n) = WB / numbit;
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
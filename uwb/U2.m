%函数1：等概率二进制的产生
function[bits]=uwb1_bits(numbits)
numbits=2;
bits=rand(1,numbits)>0.5;
%函数2：重复码编码
function[repbits]= uwb2_repcode(bits,Ns)
numbits=2;
Ns=10;
bits= uwb1_bits(numbits);
numbits=length(bits);
temprect=ones(1,Ns);
temp1=zeros(1,numbits*Ns);
temp1(1:Ns:1+Ns*(numbits-1))=bits;
temp2=conv(temp1,temprect);
repbits=temp2(1:Ns*numbits);
%函数3：脉冲形成器
function [w0]= uwb3_waveform(fc,Tm,tau);
fc=50e-9;
Tm=0.5e-9;
tau=0.25e-9;
dt=1/fc;
OVER=floor(Tm/dt);
e=mod(OVER,2);
kbk=floor(OVER/2);
tmp=linspace(dt,Tm/2,kbk);
s=(1-4.*pi.*((tmp./tau).^2)).*exp(-2.*pi.*((tmp./tau).^2));
if e
fork=1:length(s)
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
E=sum((y.^2).*dt);
w0=y./(E^0.5);
%函数4：DS编码
function [DScode]= uwb4_DS(Np);
Np=10;
DScode=((rand(1,Np)>0.5).*2)-ones(1,Np);
%函数5：PAM-DS调制
function [PAMDSseq,DSseq]=uwb5_2PAM_DS(seq,fc,Ts,DScode)
fc=50e-9;
Ts=2e-9;
Np=10;
DScode= uwb4_DS(Np);
dt=1/fc;
framesamples=floor(Ts./dt);
DSp=length(DScode);
numbits=2;
Ns=10;
bits=rand(1,numbits)>0.5;
seq= uwb2_repcode(bits,Ns);
totlength=framesamples*length(seq);
PAMDSseq=zeros(1,totlength);
DSseq=zeros(1,totlength);
for k=1:length(seq)
index=1+(k-1)*framesamples;
kDS=DScode(1+mod(k-1,DSp));
DSseq(index)=kDS;
PAMDSseq(index)=kDS*((seq(k)*2)-1);
end
%函数6：PAM-DS发射机
function[bits,DScode,Stx,ref]=uwb6_transmitter_2PAM_DS
Pow=-30;
fc=50e-9;
numbits=2;
Ts=2e-9;
Ns=10;
Np=10;
Tc=1e-9;
dpam=0.5e-9;
Nh=3;
Tm=0.5e-9;
tau=0.25e-9;
G=1;
bits= uwb1_bits(numbits);
repbits=uwb2_repcode(bits,Ns);
DScode= uwb4_DS(Np);
[PAMDSseq,DSseq]=uwb5_2PAM_DS(repbits,fc,Ts,DScode);
power=(10^(Pow/10))/1000;
Ex=power*Ts;
w0=uwb3_waveform(fc,Tm,tau);
wtx=w0.*sqrt(Ex);
Sa=conv(PAMDSseq,wtx);
Sb=conv(DSseq,wtx);
L=(floor(Ts*fc))*Ns*numbits;
Stx=Sa(1:L);
ref=Sb(1:L);
if G
F=figure(1);
set(F,'Position',[32 223 951 420]);
tmax=numbits*Ns*Ts;
time=linspace(0,tmax,length(Stx));
P=plot(time,Stx);
set(P,'LineWidth',[2]);
ylow=-1.5*max(wtx);
yhigh=1.5*max(wtx);
axis([0 tmaxylow yhigh]);
AX=gca;
set(AX,'FontSize',12);
X=xlabel('Time [s]');
set(X,'FontSize',14);
Y=ylabel('Amplitude [V]');
set(Y,'FontSize',14);
forj=1:numbits
tj=(j-1)*Ns*Ts;
L1=line([tj tj],[ylow yhigh]);
set(L1,'Color',[0 0 0],'LineStyle','--','LineWidth',[2]);
fork=0:Ns-1
ifk>0
tn=tj+k*Ts;
L2=line([tn tn],[0.8*ylow 0.8*yhigh]);
set(L2,'Color',[0.5 0.5 0.5],'LineStyle','-.','LineWidth',[1]);
end
end
end
end
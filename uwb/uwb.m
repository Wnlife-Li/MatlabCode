%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 产生二进制原信号
% 原信号比特数numbis作为输入
function [bits]=bit(numbits)
bits=rand(1,numbits)>0.5;
%rand产生的是在0～1上均匀分布的随机数
%这些数>0.5的几率各是一半，即bis为0，1的几率各半
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 产生重复编码  % 'Ns' ：码元重复数
function [repbits]=repcode(bits,Ns)
numbits = length(bits);
temprect=ones(1,Ns);
temp1=zeros(1,numbits*Ns);
temp1(1:Ns:1+Ns*(numbits-1))=bits;
temp2=conv(temp1,temprect);
repbits=temp2(1:Ns*numbits);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 引入TH码并进行PPM调制 
% 参数如下：
% 'seq'：二进制源码
% 'fc' ：抽样频率
% 'Tc' ：时隙长度
% 'Ts' ：脉冲平均重复周期
% 'dPPM'：PPM引入的时移
% 'THcode' ：TH码
% 产生两个输出：
% '2PPMTHseq' ：TH和PPM共同调制信号
% 'THseq' ：未经PPM调制的信号
function [PPMTHseq,THseq] = PPM_TH(seq,fc,Tc,Ts,dPPM,THcode)
% 调制
dt = 1 ./ fc;                   
framesamples = floor(Ts./dt);   %每个脉冲的样本数                               
chipsamples = floor (Tc./dt);                                  
PPMsamples = floor (dPPM./dt);                                 
THp = length(THcode);           

totlength = framesamples*length(seq);
PPMTHseq=zeros(1,totlength);
THseq=zeros(1,totlength);

% 引入TH码和PPM  %s(t)=sum(p(t-jTs-CjTc-aE))
for k = 1 : length(seq)
        % 脉冲位置,表示第几个脉冲-jTs
    index = 1 + (k-1)*framesamples;
        % 引入TH码,-CjTc，表示第几个时隙
    kTH = THcode(1+mod(k-1,THp));
    index = index + kTH*chipsamples;
        THseq(index) = 1;
    
    % 引入PPM时移,-aE，表示在时隙内的位置
    index = index + PPMsamples*seq(k);
    PPMTHseq(index) = 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 产生TH码
% Np:跳时码周期
% Nh:跳时码最大上界
function [THcode]=TH(Nh,Np)
THcode = floor(rand(1,Np).*Nh);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%产生UWB信号
% 参数定义如下：
% Pow:传输功率
% fc;抽样频率
% numbits：信号比特数
% Ns：每比特脉冲数
% Np:跳时码周期
% Nh:跳时码最大上界
% Ts：脉冲重复周期.
% Tc：时隙大小
% Tm:脉冲持续时间
% tau：脉冲成形因子
% dPPM：PPM引入时移
% 返回值：
% bits:产生比特流
% THcode:TH码
% Stx：产生信号
% ref：未经调制的参照信号

function [bits,THcode,Stx,ref]=transmitter_2PPM_TH(fc,numbit,Ns,Ts,dPPM)
% 输入参数
Pow = -30;       
numbits = numbit;  
Tc = 1e-9;       
Nh = 10;         
Np = 5;        
Tm = 0.5e-9;   
tau = 0.2e-9;   
G = 1;
% 模拟发射步骤
% 二进制原信号
bits = bit(numbits);
repbits = repcode(bits,Ns);  % 重复编码
THcode = TH(Nh,Np); % 产生TH码
[PPMTHseq,THseq] = PPM_TH(repbits,fc,Tc,Ts,dPPM,THcode); % 调制
% 成形滤波
power = (10^(Pow/10))/1000;                                    
Ex = power * Ts;                
w0 = waveform(fc,Tm,tau);                                
wtx = w0 .* sqrt(Ex);           
Sa = conv(PPMTHseq,wtx);                                       
Sb = conv(THseq,wtx);         
                              
% 产生输出信号
L = floor((Ts*fc))*Ns*numbits;
Stx = Sa(1:L);
ref = Sb(1:L);

if G   %绘图
F = figure(1);
set(F,'Position',[30 120 700 400]);
cla
tmax = numbits*Ns*Ts;
time = linspace(0,tmax,length(Stx));
P = plot(time,Stx);
set(P,'LineWidth',2);
ylow=-1.5*abs(min(wtx));
yhigh=1.5*max(wtx);
axis([0 tmax ylow yhigh]);
AX=gca;
set(AX,'FontSize',12);
X=xlabel('时间 [s]');
set(X,'FontSize',14);
Y=ylabel('幅度 [V]');
set(Y,'FontSize',14);
for j = 1 : numbits
    tj = (j-1)*Ns*Ts;
    L1=line([tj tj],[ylow yhigh]);
    set(L1,'Color',[0 0 0],'LineStyle', ...%间隔比特的线
       '--','LineWidth',2);
    for k = 0 : Ns-1
        if k > 0
            tn = tj + k*Nh*Tc;
            L2=line([tn tn],[ylow yhigh]);
            set(L2,'Color',[0.5 0.5 0.5],'LineStyle', ...%间隔帧的线
               '-.','LineWidth',2);
        end
        for q = 1 : Nh-1
            th = tj + k*Nh*Tc + q*Tc;
            L3=line([th th],[0.8*ylow 0.8*yhigh]);
            set(L3,'Color',[0.5 0.5 0.5],'LineStyle', ...
               ':','LineWidth',1);                       %间隔时隙的线
        end
    end
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
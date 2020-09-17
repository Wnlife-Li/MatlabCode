clear, clf
no_output_files = 0;
Ts = 0.167;
num_ch=1000;
randn('state',12);
rand('state',12);
cm =1; %选择1,2,3,4，可得到CM1,CM2,CM3,CM4四种信道
[Lam,lam,Gam,gam,nlos,sdi,sdc,sdr] = UWB_parameters(cm);
fprintf(1,['模型参数\n' ' Lam= %.4f, lam= %.4f, Gam= %.4f, gam= %.4f\n NLOS flag= %d, std_shdw= %.4f, std_ln_1= %.4f, std_ln_2= %.4f\n'],...
Lam,lam,Gam,gam,nlos,sdi,sdc,sdr);
[h_ct,t_ct,t0,np] = UWB_model_ct(Lam,lam,Gam,gam,num_ch,nlos,sdi,sdc,sdr);
[hN,N] = convert_UWB_ct(h_ct,t_ct,np,num_ch,Ts);
h = resample(hN,1,N);
h = h*N;
channel_energy = sum(abs(h).^2);
h_len = size(h,1);
t = [0:(h_len-1)]*Ts;
for k=1:num_ch
sq_h = abs(h(:,k)).^2/channel_energy(k);
t_norm = t - t0(k);
excess_delay(k) = t_norm*sq_h;
rms_delay(k) = sqrt((t_norm-excess_delay(k)).^2*sq_h);
threshold_dB = -10; % dB
temp_h = abs(h(:,k));
temp_thresh = 10^(threshold_dB/20)*max(temp_h);
num_sig_paths(k) = sum(temp_h>temp_thresh);
x = 0.85;
temp_sort = sort(temp_h.^2);
cum_energy = cumsum(temp_sort(end:-1:1));
index_e = find(cum_energy >= x*cum_energy(end), 1 ); num_sig_e_paths(k) = index_e;
end
energy_mean = mean(10*log10(channel_energy));
energy_stddev = std(10*log10(channel_energy));
mean_excess_delay = mean(excess_delay);
mean_rms_delay = mean(rms_delay);
mean_sig_paths = mean(num_sig_paths);
mean_sig_e_paths = mean(num_sig_e_paths);
fprintf(1,'Model Characteristics\n');
fprintf(1,' Mean delays: excess (tau_m) = %.1f ns, RMS (tau_rms) = %1.f\n', ...
mean_excess_delay, mean_rms_delay);
fprintf(1,' # paths: NP_10dB = %.1f, NP_85%% = %.1f\n', ...
mean_sig_paths, mean_sig_e_paths);
fprintf(1,' Channel energy: mean = %.1f dB, std deviation = %.1f dB\n', ...
energy_mean, energy_stddev);
plot(t,h), grid on
xlabel('时间 (ns)')
figure; plot([1:num_ch], excess_delay, 'b-', ...
[1 num_ch], mean_excess_delay*[1 1], 'r--' );
grid on, ylabel('过量时延(ns)'), xlabel('信道编号')
figure; plot([1:num_ch], rms_delay, 'b-', ...
[1 num_ch], mean_rms_delay*[1 1], 'r--' );
grid on, ylabel('RMS 时延扩展 (ns)'), xlabel('信道编号')
figure; plot([1:num_ch], num_sig_paths, 'b-', ...
[1 num_ch], mean_sig_paths*[1 1], 'r--');
grid on, ylabel('NP_1_0_d_B')
xlabel('信道编号')
figure; plot([1:num_ch], num_sig_e_paths, 'b-', ...
[1 num_ch], mean_sig_e_paths*[1 1], 'r--');
grid on, ylabel('NP（85%）')
xlabel('信道编号')
temp_average_power = sum(h'.*(h)')/num_ch;
temp_average_power = temp_average_power/max(temp_average_power); average_decay_profile_dB = 10*log10(temp_average_power);
figure; plot(t,average_decay_profile_dB); grid on
axis([0 t(end) -60 0]), title('平均功率时延谱')
xlabel('时延 (nsec)'), ylabel('平均功率(dB)')
figure;
figh = plot([1:num_ch],10*log10(channel_energy),'b-', ...
[1 num_ch], energy_mean*[1 1], 'g--', ...
[1 num_ch], energy_mean+energy_stddev*[1 1], 'r:', ...
[1 num_ch], energy_mean-energy_stddev*[1 1], 'r:');
xlabel('信道编号'), ylabel('dB'), title('信道能量');
legend( '每信道能量', '均值', '\pm 标准差', Best)
if no_output_files, return; end
save_fn = sprintf('cm%d_imr', cm);
save([save_fn '.mat'], 't_ct', 'h_ct', 't0', 'np', 'num_ch', 'cm');
dlmwrite([save_fn '_np.csv'], np, ',');
th_ct = zeros(size(t_ct,1),2*size(t_ct,2));
th_ct(:,1:2:end) = t_ct;
th_ct(:,2:2:end) = h_ct;
fid = fopen([save_fn '.csv'], 'w');
if fid < 0,
error('unable to write .csv file for impulse response, file may be open in another application'); end
for k = 1:size(th_ct,1)
fprintf(fid,'%.4f,%.6f,', th_ct(k,1:end));
fprintf(fid,'\r\n');
end
fclose(fid);

function [Lam,lam,Gam,gam,nlos,sdi,sdc,sdr]= UWB_parameters (cm)
tmp = 4.8/sqrt(2);
Tb2_1= [0.0233 2.5 7.1 4.3 0 3 tmp tmp; 0.4 0.5 5.5 6.7 1 3 tmp tmp;
0.0667 2.1 14.0 7.9 1 3 tmp tmp; 0.0667 2.1 24 12 1 3 tmp tmp];
Lam = Tb2_1(cm,1); lam = Tb2_1(cm,2); Gam = Tb2_1(cm,3); gam = Tb2_1(cm,4);
nlos= Tb2_1(cm,5); sdi = Tb2_1(cm,6);
sdc = Tb2_1(cm,7); sdr = Tb2_1(cm,8);
end
function [h,t,t0,np] = UWB_model_ct(Lam,lam,Gam,gam,num_ch,nlos,sdi,sdc,sdr)
sd_L = 1/sqrt(2*Lam);
sd_l = 1/sqrt(2*lam);
mu_const = (sdc^2+sdr^2)*log(10)/20;
h_len = 1000;
for k = 1:num_ch
tmp_h = zeros(h_len,1); tmp_t = zeros(h_len,1);
if nlos, Tc = (sd_L*randn)^2+(sd_L*randn)^2;
else Tc = 0;
end
t0(k) = Tc;
path_ix = 0;
while (Tc<10*Gam)
Tr = 0;
ln_xi = sdc*randn;
while (Tr<10*gam)
t_val = Tc+Tr;
mu = (-10*Tc/Gam-10*Tr/gam)/log(10) - mu_const; ln_beta = mu + sdr*randn;
pk = 2*round(rand)-1;
h_val = pk*10^((ln_xi+ln_beta)/20);
path_ix = path_ix + 1;
tmp_h(path_ix) = h_val; tmp_t(path_ix) = t_val;
Tr = Tr + (sd_l*randn)^2 + (sd_l*randn)^2; end
Tc = Tc + (sd_L*randn)^2 + (sd_L*randn)^2;
end
np(k) = path_ix;
[sort_tmp_t,sort_ix] = sort(tmp_t(1:np(k)));
t(1:np(k),k) = sort_tmp_t;
h(1:np(k),k) = tmp_h(sort_ix(1:np(k)));
fac = 10^(sdi*randn/20)/sqrt(h(1:np(k),k)'*h(1:np(k),k));
h(1:np(k),k) = h(1:np(k),k)*fac;
end
end


function [hN,N] = convert_UWB_ct(h_ct, t, np, num_channels, ts)
min_Nfs = 100; % GHz
N = max(1, ceil(min_Nfs*ts) );
N = 2^nextpow2(N);
Nfs = N/ts;
t_max = max(t(:));
h_len = 1 + floor(t_max * Nfs);
hN = zeros(h_len,num_channels);
for k = 1:num_channels
np_k = np(k);
t_Nfs = 1 + floor(t(1:np_k,k) * Nfs);
for n = 1:np_k
hN(t_Nfs(n),k) = hN(t_Nfs(n),k) + h_ct(n,k);
end
end
end
close all;

Z = d;

I=eye(2);
I1=eye(4);
O=zeros(2);
A=[I T*I;O I];
B=[0.5*(T^2)*I ;T*I];

Q=0.05*eye(2);
R=0.05*eye(4);
P=0.001 * eye(4);
u = zeros(N,2);
u(:,1) = a(:,1);
u(:,2) = a(:,2);


X=[1.6;0.51;0;0];   
X_memory=zeros(N,4);
dd=zeros(4,1);     
sins_angle_z = zeros(N,1);
 

for k=1:N
    X_ = A*X + B*u(k,:)'; 
    
    P_ = A*P*A' + B*Q*B';
 
    H = H_jacobian(X_(1),X_(2));
    dd = pos_to_dis(X_(1),X_(2));

    K=(P_*H')/(H*P_*H'+R);
    X=X_+K*(Z(k,:)' - dd');
    P=(eye(4)-K*H)*P_;
    
    X_memory(k,:)=X';
    sins_angle_z(k,1) = atan2(X(4),X(3))* 57.3;
end



figure;
subplot(2,2,1),plot((1:N),d(:,1),'b');grid on,xlabel('time/T'),ylabel('m'),title('d0'),legend('d0');%,(1:N),d_exp(:,1),'r','d0 ground truth'
subplot(2,2,2),plot((1:N),d(:,2),'b');grid on,xlabel('time/T'),ylabel('m'),title('d1'),legend('d1');%,(1:N),d_exp(:,2),'r','d1 ground truth'
subplot(2,2,3),plot((1:N),d(:,3),'b');grid on,xlabel('time/T'),ylabel('m'),title('d2'),legend('d2');%,(1:N),d_exp(:,3),'r','d2 ground truth'
subplot(2,2,4),plot((1:N),d(:,4),'b');grid on,xlabel('time/T'),ylabel('m'),title('d3'),legend('d3');%,(1:N),d_exp(:,4),'r' ,'d3 ground truth'


figure;
subplot(2,1,1),plot((1:N),X_wls2d(:,1),'g',(1:N),toa_kf_pv2d(:,1),'m',(1:N),X_memory(:,1),'b',(1:N),a_position(:,1),'k');grid on,xlabel('time/T'),ylabel('m'),title('pos.x'),legend('pos.x','kf pos.x','uwb/ahrs pos.x','acc pos.x');%,(1:N),pos_exp(:,1),'r'  , 'pos.x ground truth'
subplot(2,1,2),plot((1:N),pos(:,1)-pos_exp(:,1),'g',(1:N),toa_kf_pv2d(:,1)-pos_exp(:,1),'m',(1:N),X_memory(:,1)-pos_exp(:,1),'b');grid on,xlabel('time/T'),ylabel('m'),title('pos.x error'),legend('pos.x error','kf pos.x error','uwb/ahrs pos.x error');

figure;
subplot(2,1,1),plot((1:N),X_wls2d(:,2),'g',(1:N),toa_kf_pv2d(:,2),'m',(1:N),X_memory(:,2),'b',(1:N),a_position(:,2),'k');grid on,xlabel('time/T'),ylabel('m'),title('pos.y'),legend('pos.y','kf pos.y','uwb/ahrs pos.y','acc pos.y');%,(1:N),pos_exp(:,2),'r' ,'pos.y ground truth'
subplot(2,1,2),plot((1:N),pos(:,2)-pos_exp(:,2),'g',(1:N),toa_kf_pv2d(:,2)-pos_exp(:,2),'m',(1:N),X_memory(:,2)-pos_exp(:,2),'b');grid on,xlabel('time/T'),ylabel('m'),title('pos.y error'),legend('pos.y error','kf pos.y error','uwb/ahrs pos.y error');
figure;
subplot(2,1,1),plot((1:N),toa_kf_pv2d(:,3),'m',(1:N),X_memory(:,3),'b',(1:N),a_velocity(:,1),'k');grid on,xlabel('time/T'),ylabel('m/s'),title('vel.x'),legend('kf vel.x','uwb/ahrs vel.x','acc vel.x');%,(1:N),vel_exp(:,1),'r' ,'vel.x ground truth'
subplot(2,1,2),plot((1:N),toa_kf_pv2d(:,3)-vel_exp(:,1),'m',(1:N),X_memory(:,3)-vel_exp(:,1),'b');grid on,xlabel('time/T'),ylabel('m/s'),title('vel.x error'),legend('kf vel.x error','uwb/ahrs vel.x error' );

figure;
subplot(2,1,1),plot((1:N),toa_kf_pv2d(:,4),'m',(1:N),X_memory(:,4),'b',(1:N),a_velocity(:,2),'k');grid on,xlabel('time/T'),ylabel('m/s'),title('vel.y'),legend('kf vel.y','uwb/ahrs vel.y','acc vel.y');%,(1:N),vel_exp(:,2),'r','vel.y ground truth'
subplot(2,1,2),plot((1:N),toa_kf_pv2d(:,4)-vel_exp(:,2),'m',(1:N),X_memory(:,4)-vel_exp(:,2),'b');grid on,xlabel('time/T'),ylabel('m/s'),title('vel.y error'),legend('kf vel.y error','uwb/ahrs vel.y error' );


figure;
plot(X_wls2d(:,1),X_wls2d(:,2),'g',toa_kf_pv2d(:,1),toa_kf_pv2d(:,2),'m',X_memory(:,1),X_memory(:,2),'b','LineWidth',2);xlabel('x/m'),ylabel('y/m'),title('2D trajectory');grid on;legend('pos','kf','uwb/ahrs');%pos_exp(:,1),pos_exp(:,2),'r', ,'ground truth' ,axis([0 2 0 6])


%figure;
%plot((1:N),toa_angle_z(:,1),'g',(1:N),sins_angle_z(:,1),'b',(1:N),angle_z_exp,'r');grid on,xlabel('time/T'),ylabel('angle.z'),title('angle.z'),legend('angle.z','angle ground truth.z');
RMSE_Pos_X = rms(X_wls2d(:,1)-pos_exp(:,1));
RMSE_Pos_Y = rms(X_wls2d(:,2)-pos_exp(:,2));
KF_Pos_X = rms(toa_kf_pv2d(:,1)-pos_exp(:,1));
KF_Pos_Y = rms(toa_kf_pv2d(:,2)-pos_exp(:,2));
UWB_AHRS_Pos_X = rms(X_memory(:,1)-pos_exp(:,1));
UWB_AHRS_Pos_Y = rms(X_memory(:,2)-pos_exp(:,2));

CEP_UWB_AHRS = 0.598*(std(X_memory(:,1)-pos_exp(:,1))+std(X_memory(:,2)-pos_exp(:,2)));
CEP95_UWB_AHRS = 1.2272*(std(X_memory(:,1)-pos_exp(:,1))+std(X_memory(:,2)-pos_exp(:,2)));
CEP99_UWB_AHRS = 1.5222*(std(X_memory(:,1)-pos_exp(:,1))+std(X_memory(:,2)-pos_exp(:,2)));

CEP99_KF = 1.5222*(std(toa_kf_pv2d(:,1)-pos_exp(:,1))+std(toa_kf_pv2d(:,2)-pos_exp(:,2)));
CEP99_Pos = 1.5222*(std(pos(:,1)-pos_exp(:,1))+std(pos(:,2)-pos_exp(:,2)));




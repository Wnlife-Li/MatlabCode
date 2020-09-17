% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ?0 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Start of script

addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data

% load('ExampleData.mat');

newData = importdata('ahrs3.tsv', '\t', 2);
data=newData.data;
[np,mp]=size(data);
Gyroscope=zeros(np,3);
Accelerometer=zeros(np,3);
Magnetometer=zeros(np,3);
jiaodu=zeros(np,3);
for i=1:3
   Gyroscope(:,i)=data(:,i+4); 
   Accelerometer(:,i)=data(:,i+1);
   Magnetometer(:,i)=data(:,i+11);
   jiaodu(:,i)=data(:,i+7);
end
%%根据椭球拟合算法，对磁力计的数据进行预处理
Magnetometer(:,2)=Magnetometer(:,2)*0.879366495887078;
Magnetometer(:,3)=Magnetometer(:,3)*0.753356264866312;

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(Gyroscope(:,1), 'r');
plot(Gyroscope(:,2), 'g');
plot(Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(Accelerometer(:,1), 'r');
plot(Accelerometer(:,2), 'g');
plot(Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(Magnetometer(:,1), 'r');
plot(Magnetometer(:,2), 'g');
plot(Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/50, 'Beta', 0.1,'Quaternion',[1 -0.1011 -0.0567 0]);
% AHRS = MadgwickAHRS('SamplePeriod', 1/50, 'Beta', 0.1);

% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(Accelerometer), 4);
for t = 1:length(Accelerometer)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
%     AHRS.UpdateIMU(Gyroscope(t,:) * (pi/180), Accelerometer(t,:));
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ?0
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot( euler(:,1), 'r');
plot( euler(:,2), 'g');
plot( euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;
figure('Name', 'Jiaodu');
hold on;
plot( jiaodu(:,1), 'r');
plot( jiaodu(:,2), 'g');
plot( jiaodu(:,3), 'b');
title('jiaodu');


%% End of script
%%  使用加速度的Gauss-Newton method生成初试四元数
clc;
clear;
format long; 
syms q0 q1 q2 q3;
%x,y,z代表ax，ay，az
ax=0.1133;
ay=-0.2021;
az=0.9937;
q0=1;
q1=0;
q2=0;
q3=0;
error=1e-5;   %停机门限
f = [2*(q1*q3-q0*q2)-ax; 2*(q0*q1+q2*q3)-ay; 2*(1/2-q1*q1-q2*q2)-az];
q=[q0;q1;q2;q3];
J=[-2*q2,2*q3,-2*q0,2*q1;2*q1,2*q0,2*q3,2*q2;0,-4*q1,-4*q2,0];

F=f'*f;
k=0;
tic
while(F>error)
    d=-pinv(J'*J)*J'*f;
    q=q+d;
    q0=q(1);
    q1=q(2);
    q2=q(3);
    q3=q(4);
    J=[-2*q2,2*q3,-2*q0,2*q1;2*q1,2*q0,2*q3,2*q2;0,-4*q1,-4*q2,0];  
    f = [2*(q1*q3-q0*q2)-ax; 2*(q0*q1+q2*q3)-ay; 2*(1/2-q1*q1-q2*q2)-az];
    F=f'*f;
    k=k+1; 
end
disp('Gauss-Newton algorithm');
 toc
 k
 q
 F
 


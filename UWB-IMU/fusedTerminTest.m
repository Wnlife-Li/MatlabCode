clc;
clear all;

x=zeros(13,2);
x(1,:)=[1,1];
for i=2:6
    x(i,1)=x(i-1,1)+0.6;
    x(i,2)=1; 
end
for i=7:8
    x(i,1)=x(i-1,1);
    x(i,2)=x(i-1,2)+0.6;
end
for i=9:13
   x(i,1)=x(i-1,1)-0.6;
   x(i,2)=x(i-1,2);
end


plot(x);

clc;
clear all;
preDealData=zeros(50,3);
%1m
temp1=importdata('1m.txt')./1000;
preDealData(:,1)=temp1(:,1);
%2m
temp2=importdata('2m.txt')./1000;
preDealData(:,2)=temp2(:,1);
%3m
temp3=importdata('3m.txt')./1000;
preDealData(:,3)=temp3(:,1);

%均值
meanLos=zeros(1,3);
for i=1:3
   meanLos(1,i)=mean(preDealData(:,i)); 
end

%误差
errorLos=zeros(50,3);
for i=1:3
    errorLos(:,i)=preDealData(:,i)-i;
end

%平均误差
meanError=zeros(1,3);
for i=1:3
   meanError(1,i)=mean(errorLos(:,i)); 
end

%最大误差
maxError=zeros(1,3);
for i=1:3
   maxError(1,i)=max(errorLos(:,i)); 
end

%方差
s=S(preDealData,meanLos);

clc;
clear all;
preDealData=zeros(50,6);
%1m
temp1=importdata('1m.txt')./1000;
preDealData(:,1)=temp1(:,1);
%1.5m
temp1_5=importdata('1.5m.txt')./1000;
preDealData(:,2)=temp1_5(:,1);
%2m
temp2=importdata('2m.txt')./1000;
preDealData(:,3)=temp2(:,1);
%2.5m
temp2_5=importdata('2.5m.txt')./1000;
preDealData(:,4)=temp2_5(:,1);
%3m
temp3=importdata('3m.txt')./1000;
preDealData(:,5)=temp3(:,1);
%3.5m
temp3_5=importdata('3.5m.txt')./1000;
preDealData(:,6)=temp3_5(:,1);

%均值
meanLos=zeros(1,6);
for i=1:6
   meanLos(1,i)=mean(preDealData(:,i)); 
end

%误差
errorLos=zeros(50,6);
k=1;
for i=1:0.5:3.5
    errorLos(:,k)=preDealData(:,k)-i;
    k=k+1;
end

%平均误差
meanError=zeros(1,6);
for i=1:6
   meanError(1,i)=mean(errorLos(:,i)); 
end

%最大误差
maxError=zeros(1,6);
for i=1:6
   maxError(1,i)=max(errorLos(:,i)); 
end

s=S(preDealData,meanLos);


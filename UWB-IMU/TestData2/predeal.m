%数据预处理
clc;
clear all;
lastdata=zeros(35,4);
for i=1:35
    str=sprintf('%s%d%s','F:\Matlab\R2016\bin\UWB-IMU\TestData2\测试数据\',i,'.txt');
    data = importdata(str);
    newdata=data./1000;
    a=mean(newdata,1);
    lastdata(i,:)=a;
end
%将数据写入到txt文件
fid=fopen('x.txt','wt');%写入文件路径
matrix=lastdata;                       %input_matrix为待输出矩阵
   [m,n]=size(matrix);
  for i=1:1:m
     for j=1:1:n
        if j==n
         fprintf(fid,'%.4f\n',matrix(i,j));
        else 
         fprintf(fid,'%.4f\t',matrix(i,j));
        end
     end
 end
 fclose(fid);
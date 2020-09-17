clc;
clear all;
realPos=zeros(35,2);%初始化真实位置
realPos(1,:)=[9,10.2];
for i=2:13
   realPos(i,1)=9; 
   realPos(i,2)=10.2-0.6*(i-1);
end
k=1;
for i=14:23
    realPos(i,1)=9-0.6*k; 
    realPos(i,2)=3; 
    k=k+1;
end
j=1;
for i=24:35
    realPos(i,1)=3; 
    realPos(i,2)=3+0.6*j; 
    j=j+1;
end

%真实坐标数据存储到txt文件
% fid=fopen('r_p.txt','wt');%写入文件路径
% matrix=realPos;                       %input_matrix为待输出矩阵
%    [m,n]=size(matrix);
%   for i=1:1:m
%      for j=1:1:n
%         if j==n
%          fprintf(fid,'%.4f\n',matrix(i,j));
%         else 
%          fprintf(fid,'%.4f\t',matrix(i,j));
%         end
%      end
%  end
%  fclose(fid);
 
%绝对行走距离测量值
realdata=zeros(35,4);
for i=1:35
   realdata(i,1)=sqrt((realPos(i,1)- 9)^2+(realPos(i,2)- 10.8)^2);
   realdata(i,2)=sqrt((realPos(i,1)- 9)^2+(realPos(i,2)- 0.75)^2);
   realdata(i,3)=sqrt((realPos(i,1)- 3)^2+(realPos(i,2)- 0.75)^2);
   realdata(i,4)=sqrt((realPos(i,1)- 3)^2+(realPos(i,2)- 10.8)^2); 
end
%将绝对行走距离测量值保存
fid=fopen('r_d.txt','wt');%写入文件路径
matrix=realdata;                       %input_matrix为待输出矩阵
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

 
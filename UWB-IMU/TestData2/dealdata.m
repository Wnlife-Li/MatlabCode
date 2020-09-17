x_data = importdata('x.txt');
r_data = importdata('r_d.txt');

%矫正A0数据
x_data(8,1)=x_data(8,1)+0.1;
x_data(9,1)=x_data(9,1)+0.1;
x_data(10,1)=x_data(10,1)+0.3;
x_data(11,1)=x_data(11,1)+0.2;
for i=12:31
    x_data(i,1)=x_data(i,1)+0.4;
end
x_data(13,1)=x_data(13,1)+0.1;

x_data(32,1)=x_data(32,1)+0.2;
x_data(33,1)=x_data(33,1)+0.2;
x_data(34,1)=x_data(34,1)+0.2;
x_data(35,1)=x_data(35,1)+0.3;
for i=1:35
    x_data(i,1)=x_data(i,1)+0.2;
end

%矫正A1的数据
for i=1:3
    x_data(i,2) = x_data(i,2)+0.3;
end
x_data(4,2) = x_data(4,2)+0.7;
x_data(5,2) = x_data(5,2)+0.5;
x_data(7,2) = x_data(7,2)+0.5;
x_data(9,2) = x_data(9,2)+0.5;
x_data(15,2) = x_data(15,2)+0.3;
for i=16:20
    x_data(i,2) = x_data(i,2)+0.5;
end
x_data(21,2) = x_data(21,2)+0.3;
for i=22:24
    x_data(i,2) = x_data(i,2)+0.45;
end 
x_data(25,2) = x_data(25,2)+0.4; 
x_data(26,2) = x_data(26,2)+1.2;  
for i=27:30
    x_data(i,2) = x_data(i,2)+0.8;
end 
x_data(31,2) = x_data(31,2)+1.15;
x_data(32,2) = x_data(32,2)+0.5;
x_data(33,2) = x_data(33,2)+0.6;
x_data(34,2) = x_data(34,2)+1;
x_data(35,2) = x_data(35,2)+1.6;
 
%矫正A2的数据
for i=1:8
    x_data(i,3) = x_data(i,3)+0.95;
end
x_data(1,3) = x_data(1,3)+0.1;
x_data(9,3) = x_data(9,3)+0.6;
x_data(12,3) = x_data(12,3)+0.6;
x_data(15,3) = x_data(15,3)+0.6;
x_data(19,3) = x_data(19,3)+0.2;
x_data(20,3) = x_data(20,3)+0.4;
x_data(23,3) = x_data(23,3)+0.4;
x_data(24,3) = x_data(24,3)+0.45;
for i=25:28
    x_data(i,3) = x_data(i,3)+0.5;
end
for i=29:35
    x_data(i,3) = x_data(i,3)+0.7;
end

%矫正A3的数据
for i=1:4
    x_data(i,4) = x_data(i,4)+0.2;
end
for i=5:8
    x_data(i,4) = x_data(i,4)+0.3;
end
for i=9:11
    x_data(i,4) = x_data(i,4)+0.4;
end
x_data(12,4) = x_data(12,4)+0.25;
x_data(13,4) = x_data(13,4)+0.5;
x_data(14,4) = x_data(14,4)+0.3;
for i=15:19
    x_data(i,4) = x_data(i,4)+0.4;
end
x_data(20,4) = x_data(20,4)+0.3;
x_data(21,4) = x_data(21,4)+0.4;
for i=23:25
    x_data(i,4) = x_data(i,4)+0.4;
end 
x_data(26,4) = x_data(26,4)+0.3;
x_data(27,4) = x_data(27,4)+0.3;
x_data(28,4) = x_data(28,4)+0.2;

 
A0=x_data(:,1)-r_data(:,1);
A1=x_data(:,2)-r_data(:,2);
A2=x_data(:,3)-r_data(:,3);
A3=x_data(:,4)-r_data(:,4);

figure;
plot(x_data(:,1),'r');
hold on;
plot(r_data(:,1),'g');
title('A0');
figure;
plot(A0,'-*');
 
figure;
plot(x_data(:,2),'r');
hold on;
plot(r_data(:,2),'g');
title('A1');
figure;
plot(A1,'-*');

figure;
plot(x_data(:,3),'r');
hold on;
plot(r_data(:,3),'g');
title('A2');
figure;
plot(A2,'-*');

figure;
plot(x_data(:,4),'r');
hold on;
plot(r_data(:,4),'g');
title('A3');
figure;
plot(A3,'-*');


%将矫正后的测量UWB数据写入到r_x.txt文件
% fid=fopen('r_x.txt','wt');%写入文件路径
% matrix=x_data;                      
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



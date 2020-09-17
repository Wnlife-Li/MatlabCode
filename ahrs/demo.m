newData = importdata('ahrs.tsv', '\t', 2);
data=newData.data;
[np,mp]=size(data);
Gyroscope=zeros(np,3);
Accelerometer=zeros(np,3);
Magnetometer=zeros(np,3);
for i=1:3
   Gyroscope(:,i)=data(:,i+4); 
   Accelerometer(:,i)=data(:,i+1);
   Magnetometer(:,i)=data(:,i+11);
end
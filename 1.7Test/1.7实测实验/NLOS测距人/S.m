function [ req ] = S( data,mean )
req=zeros(1,3);
 for i=1:3
     req(i)=sum((data(:,i)-mean(i)).^2)/50;
 end
end
function [ req ] = S( data,mean )
req=zeros(1,6);
 for i=1:6
     req(i)=sum((data(:,i)-mean(i)).^2)/50;
 end
end


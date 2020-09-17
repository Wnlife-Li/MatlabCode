function [ req ] = S( data,mean )
     req=sum((data-mean).^2)/200;
end
function [ d ] = pos_to_d( x,y )

% dis(1) = sqrt(x^2+y^2);
% dis(2) = sqrt(x^2+(y-6)^2);
% dis(3) = sqrt((x-6)^2+(y-6)^2);
% dis(4) = sqrt((x-6)^2+y^2);

d(1) = sqrt(x^2+y^2);
d(2) = sqrt(x^2+(y-4.725)^2);
d(3) = sqrt((x-3.53)^2+(y-4.725)^2);
d(4) = sqrt((x-3.53)^2+y^2);

end

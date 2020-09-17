function [ dis ] = pos_to_dis( x,y )

% dis(1) = sqrt(x^2+y^2);
% dis(2) = sqrt(x^2+(y-6)^2);
% dis(3) = sqrt((x-6)^2+(y-6)^2);
% dis(4) = sqrt((x-6)^2+y^2);

dis(1) = sqrt(x^2+y^2);
dis(2) = sqrt(x^2+(y-4.725)^2);
dis(3) = sqrt((x-3.53)^2+(y-4.725)^2);
dis(4) = sqrt((x-3.53)^2+y^2);

end





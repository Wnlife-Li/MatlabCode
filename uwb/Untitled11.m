Tp = 1e-8;           % pulse duration [s]
Nc = 8; 
A = 1;               % pulse amplitude [V]

smp = 256;          % number of samples for representing the pulse

% --------------------------------------------
% Step One - Generation of the reference pulse
% --------------------------------------------

f = Nc / Tp;
p = sin(2.*pi.*f.*linspace(0,Tp,smp));
plot(linspace(0,Tp,smp),p);
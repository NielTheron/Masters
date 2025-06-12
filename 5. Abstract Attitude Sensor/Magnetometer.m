%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The Purpose of this function is to abstract a magnetometer
%=========================================================================
function MAG = Magnetometer(attitude,MAG_noise)

rad_mag = deg2rad(MAG_noise);
u = randn(3,1);
u = u / norm(u); % Normalise
q_mag = [cos(rad_mag/2); u*sin(rad_mag/2)];
MAG = quatmultiply(q_mag.',attitude.').';

end

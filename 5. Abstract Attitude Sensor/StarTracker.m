%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The Purpose of this function is to abstract a Star Tracker
%=========================================================================
function ST = StarTracker(attitude,ST_noise)

rad_st = deg2rad(ST_noise);
u = randn(3,1);
u = u / norm(u); % Normalize
q_st = [cos(rad_st/2); u*sin(rad_st/2)];
ST = quatmultiply(q_st.',attitude.').';

end
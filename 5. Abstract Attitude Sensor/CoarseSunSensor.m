%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The Purpose of this function is to abstract a coarse sun sensor
%=========================================================================
function z_CSS = CoarseSunSensor(attitude,CSS_noise)

rad_css = deg2rad(CSS_noise);
u = randn(3,1);
u = u / norm(u);  % Normalize
q_css = [cos(rad_css/2); u*sin(rad_css/2)];
z_CSS = quatmultiply(q_css.',attitude.');

end


%==========================================================================
% Niel Theron
% 21-05-2025
%==========================================================================
% The purpose of this function is to propagate the "GROUND TRUTH" orbit
% This function can also be tuned to add more or less realism.
%==========================================================================
% INPUT:
% x         : Current state
% dt        : Sample rate (s)
% I         : Moment of inertia (kg*m2)
% Mu        : The gravitational constant of the Earth (km3/s2)
% Re        : Radius of Earth
% J2        : J2 harmonic constant
% 
% OUTPUT:
% xp        : Propogated state
%
% VARAIBLES:
% R         : Magnitude of position vector
% C         : Gracitational acceleration constant
% z2        : Z-axis component for J2 calculation
% r2        : R component for J2 calcuation
% J         : J2 constant
% [ax_J2 ay_J2 az_J2]       : J2 acceleration
%==========================================================================
function xp = Plant(x,dt,I,Mu,Re,J2)

% Initialise Varaibles
xp = zeros(13,1);
%---

% Calcualte the gravitaional effect
R = norm(x(1:3));
C = -Mu/R^3;
%---

% Work out the effect of J2
z2 = x(3)^2;
r2 = R^2;
J = 1.5 * J2 * Mu * Re^2 / R^5;
ax_J2 = J * x(1) * (5*z2/r2 - 1);
ay_J2 = J * x(2) * (5*z2/r2 - 1);
az_J2 = J * x(3) * (5*z2/r2 - 3);
%---

% Normalise Quaternion
x(7:10) = quatnormalize(x(7:10).');
%---

% Caculate the next State
xp(1,1)  = x(1) + x(4)*dt;                                         % rx
xp(2,1)  = x(2) + x(5)*dt;                                         % ry
xp(3,1)  = x(3) + x(6)*dt;                                         % rz
xp(4,1)  = x(4) + (C*x(1) + ax_J2)*dt;                             % vx
xp(5,1)  = x(5) + (C*x(2) + ay_J2)*dt;                             % vy
xp(6,1)  = x(6) + (C*x(3) + az_J2)*dt;                             % vz
xp(7,1)  = x(7) + 0.5*(-x(11)*x(8)-x(12)*x(9)-x(13)*x(10))*dt;     % qs
xp(8,1)  = x(8) + 0.5*( x(11)*x(7)+x(13)*x(9)-x(12)*x(10))*dt;     % qx
xp(9,1)  = x(9) + 0.5*( x(12)*x(7)-x(13)*x(8)+x(11)*x(10))*dt;     % qy
xp(10,1) = x(10)+ 0.5*( x(13)*x(7)+x(12)*x(8)+x(11)*x(9))*dt;      % qz
xp(11,1) = x(11)+ ((I(2)-I(3))*x(12)*x(13))/I(1)*dt;               % wx
xp(12,1) = x(12)+ ((I(3)-I(1))*x(13)*x(11))/I(2)*dt;               % wy
xp(13,1) = x(13)+ ((I(1)-I(2))*x(11)*x(12))/I(3)*dt;               % wz
%---

% Normalise quaternion
xp(7:10) = quatnormalize(xp(7:10).');
%---
end
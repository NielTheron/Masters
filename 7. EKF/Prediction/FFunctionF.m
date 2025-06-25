%==========================================================================
% Niel Theron
% 20-03-2025
%==========================================================================
% The Purpose of this function is determine the next state in the system
% using non-linear equations
% I         : Moment of Inertia
% Mu        : The gravitational constant of the Earth
% R         : Magnitude of the distance vector
% x = xn    : Current States
% f         : Output of the F_function
%==========================================================================
function f = FFunctionF(x,I,dt,Mu,Re,J2)

% Initialise Varaibles
f = zeros(13,1);
%---

% Calcualte the gravitaional effect
R = norm(x(1:3));
C = -Mu/R^3;
%---

% Work out the effect of J2
z2 = x(3)^2;
r2 = R^2;
J2_factor = 1.5 * J2 * Mu * Re^2 / R^5;
ax_J2 = J2_factor * x(1) * (5*z2/r2 - 1);
ay_J2 = J2_factor * x(2) * (5*z2/r2 - 1);
az_J2 = J2_factor * x(3) * (5*z2/r2 - 3);
%---

% Normalise Quaternion
x(7:10) = quatnormalize(x(7:10).');
%---
% Caculate the next State
f(1,1)  = x(1) + x(4)*dt;                                         % rx
f(2,1)  = x(2) + x(5)*dt;                                         % ry
f(3,1)  = x(3) + x(6)*dt;                                         % rz
f(4,1)  = x(4) + (C*x(1) + ax_J2)*dt;                             % vx
f(5,1)  = x(5) + (C*x(2) + ay_J2)*dt;                             % vy
f(6,1)  = x(6) + (C*x(3) + az_J2)*dt;                             % vz
f(7,1)  = x(7) + 0.5*(-x(11)*x(8)-x(12)*x(9)-x(13)*x(10))*dt;     % qs
f(8,1)  = x(8) + 0.5*( x(11)*x(7)+x(13)*x(9)-x(12)*x(10))*dt;     % qx
f(9,1)  = x(9) + 0.5*( x(12)*x(7)-x(13)*x(8)+x(11)*x(10))*dt;     % qy
f(10,1) = x(10)+ 0.5*( x(13)*x(7)+x(12)*x(8)+x(11)*x(9))*dt;      % qz
f(11,1) = x(11)+ ((I(2)-I(3))*x(12)*x(13))/I(1)*dt;               % wx
f(12,1) = x(12)+ ((I(3)-I(1))*x(13)*x(11))/I(2)*dt;               % wy
f(13,1) = x(13)+ ((I(1)-I(2))*x(11)*x(12))/I(3)*dt;               % wz
%---

f(7:10) = quatnormalize(f(7:10).');
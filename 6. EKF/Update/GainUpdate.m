%==========================================================================
% Niel Theron
% 20-03-2025
%==========================================================================
% The purpose of this function is to calculate the Kalman gain of the
% system
%==========================================================================
% xm = x_{n,n-1}    : State estimate
% d = dn            : Catalogue vector
% Pm = P_{n,n-1}    : Previous covariance matrix
% I = I             : Moment of inertia vector
% H = dh/dx         : Measurement jacobian
% R = R             : Measurement noise
%==========================================================================
function K = GainUpdate(H,Pm,R)
K = Pm*H.'/(H*Pm*H.' + R);
end
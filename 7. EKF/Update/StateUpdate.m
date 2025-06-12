%==========================================================================
% Niel Theron
% 03-08-2025
%==========================================================================
% The Purpose of this function is to update the state of the
% system
% x = x_{n,n}       : Current state estimate
% K = Kn            : Current Kalman gain
% xm = x_{n,n-1}    : Previous state sstimate
% z = zn            : Current measurement
%==========================================================================
function [xm,y_est] = StateUpdate(xm,K,z,c)
y_est = HFunction(xm,c);
xm = xm + K*(z-y_est);
end
%==========================================================================
% Niel Theron
% 20-03-2025
%==========================================================================
% The purpose of this function is to update the covariance matrix.
%==========================================================================
% P = P_{n,n}   : Current covariance matrix
% K = K-n       : Current Kalman gain
% H = dh/dx     : Measurement jacobian
% R = R         : Measurement Noise
% Pm = P_{n,n-1}: Previous covariance matrix
% x = x_{n,n}   : Current state
%==========================================================================
function P = CovarianceUpdate(K,Pm,R,H)

I = eye(size(Pm));
P = (I-K*H)*Pm*(I-K*H).' + K*R*K.';

end
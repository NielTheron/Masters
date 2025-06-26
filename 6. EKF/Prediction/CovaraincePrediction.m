%==========================================================================
% Niel Theron
% 20-03-2025
%==========================================================================
% The purpose of this function is to predict the next timestep covariance
% matrix.
%==========================================================================
% Pp = P_{n+1,n}        : Next covariance matrix
% F = df/dx             : Model jacobian
% P = P_{n,1}           : Current covariance matrix
% Q = Q_n               : Current process noise
% dt                    : Sample time
% I                     : Moment of inertia
%==========================================================================

function Pp = CovaraincePrediction(x,P,Q,dt,I,Mu,Re,J2)

F = FJacob_numerical(x,I,dt,Mu,Re,J2);

Pp = F*P*F.' + Q;

end
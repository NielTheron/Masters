%==========================================================================
% Niel Theron
% 08-03-2025
%==========================================================================
% The purpose of this function is to predict the next state based on the
% current state
% xp = x_{n+1,n}    : Next State Estimate
% F                 : System Model
% x = x_{n,n}       : Current State Estimate
% G                 : Input Matrix
% u = u_n           : Current Input
%==========================================================================
function xp = StatePredictionF(x,I,dt,Mu,Re,J2)

% Calcualte next state
f = FFunctionF(x,I,dt,Mu,Re,J2);
xp = f;
%---

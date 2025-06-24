%==========================================================================
% Niel Theron
% 20-03-2025
%==========================================================================
% This is the Main EKF function
% The purpose of this function is to estimate the state vector of the
% system
%==========================================================================
% y_est         : Estimated measurement from the estimated state vector
% x_est         : Estimated state
% P_est         : Co-varaince matrix for the extimated state)
% n_z           : Number of measured states
% n_x           : Number of states in the statevector
% K_est         : Kalman Gain
% c             : Internal catalogue vector.
%==========================================================================
function [y_ET, x_EKF, P_EKF, K_ET] ...
 = EKF( ...
 catalogue,x_EKF,P_EKF,I_f,Q_f,dt_f,Mu_f, ...
 z_ET, z_CSS, z_MAG, z_ST, z_GPS, z_GYR, ...
 R_ET, R_CSS, R_MAG, R_ST, R_GPS, R_GYR)

%% Initialise Varaibles
n_z = 3;
n_x = 13;
n_f = size(z_ET,2);
y_ET = zeros(n_z,n_f);
K_ET = zeros(n_x,n_z,n_f);
%---

%% Prediction step
xp_EKF = StatePredictionF(x_EKF,I_f,dt_f,Mu_f);
Pp_EKF = CovaraincePrediction(xp_EKF,P_EKF,Q_f,dt_f,I_f,Mu_f);
%---


%% Update step

% 1. Coarse Sun Sensor (lowest accuracy first)
if norm(z_CSS) ~= 0
    H = [0 0 0 0 0 0 1 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 0 0 0 0 0;
         0 0 0 0 0 0 0 0 1 0 0 0 0;
         0 0 0 0 0 0 0 0 0 1 0 0 0];
    K_Att = GainUpdate(H,Pp_EKF,R_CSS);
    xp_EKF = StateUpdateAttitude(xp_EKF,K_Att,z_CSS);
    Pp_EKF = CovarianceUpdate(K_Att,Pp_EKF,R_CSS,H);
end
%---

% 2. Magnetometer
% if norm(z_MAG) ~= 0
%     H = [0 0 0 0 0 0 1 0 0 0 0 0 0;
%          0 0 0 0 0 0 0 1 0 0 0 0 0;
%          0 0 0 0 0 0 0 0 1 0 0 0 0;
%          0 0 0 0 0 0 0 0 0 1 0 0 0];
%     K_Att = GainUpdate(H,Pp_EKF,R_MAG);
%     xp_EKF = StateUpdateAttitude(xp_EKF,K_Att,z_MAG);
%     Pp_EKF = CovarianceUpdate(K_Att,Pp_EKF,R_MAG,H);
% end
% ---

% % 3. Earth Tracker (position depends on measured accuracy)
% % Debug Script: Add this to your EKF update loop to find the problem
% % Insert this right before your EKF update for the first feature
% 
% for i = 1:n_f
% 
%     if norm(z_ET(:,i)) ~= 0
%         H_ET = Improved_Numerical_H(xp_EKF,catalogue(:,i));
%         [K_ET(:,:,i)] = GainUpdate(H_ET,Pp_EKF,R_ET);
%         [xp_EKF,y_ET(:,i)] = StateUpdate(xp_EKF,K_ET(:,:,i),z_ET(:,i),catalogue(:,i));
%         Pp_EKF = CovarianceUpdate(K_ET(:,:,i),Pp_EKF,R_ET,H_ET);
%         xp_EKF(7:10) = quatnormalize(xp_EKF(7:10).');
%     end
% end
% %---

% 4. Gyroscope
if norm(z_GYR) ~= 0
    H = [0 0 0 0 0 0 0 0 0 0 1 0 0;
         0 0 0 0 0 0 0 0 0 0 0 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 1];
    K_AngV= GainUpdate(H,Pp_EKF,R_GYR);
    xp_EKF = StateUpdateAngV(xp_EKF,K_AngV,z_GYR);
    Pp_EKF = CovarianceUpdate(K_AngV,Pp_EKF,R_GYR,H);
end
%---
% 
% 5. GPS (high position accuracy)
if norm(z_GPS) ~= 0
    H = [1 0 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0 0 0 0 0];
    K_Pos = GainUpdate(H,Pp_EKF,R_GPS);
    xp_EKF = StateUpdatePosition(xp_EKF,K_Pos,z_GPS);
    Pp_EKF = CovarianceUpdate(K_Pos,Pp_EKF,R_GPS,H);
end
%---

% % 6. Star Tracker (highest accuracy, final correction)
% if norm(z_ST) ~= 0
%     H = [0 0 0 0 0 0 1 0 0 0 0 0 0;
%          0 0 0 0 0 0 0 1 0 0 0 0 0;
%          0 0 0 0 0 0 0 0 1 0 0 0 0;
%          0 0 0 0 0 0 0 0 0 1 0 0 0];
%     K_Att = GainUpdate(H,Pp_EKF,R_ST);
%     xp_EKF = StateUpdateAttitude(xp_EKF,K_Att,z_ST);
%     Pp_EKF = CovarianceUpdate(K_Att,Pp_EKF,R_ST,H);
% end
% %---

x_EKF = xp_EKF;
P_EKF = Pp_EKF;


end


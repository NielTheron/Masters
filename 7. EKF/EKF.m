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
function [y_ET, xp_EKF, Pp_EKF, K_ET] ...
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

%% Update step

% 1. Coarse Sun Sensor (lowest accuracy first)
if norm(z_CSS) ~= 0
    H = [0 0 0 0 0 0 1 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 0 0 0 0 0;
         0 0 0 0 0 0 0 0 1 0 0 0 0;
         0 0 0 0 0 0 0 0 0 1 0 0 0];
    K_Att = GainUpdate(H,P_EKF,R_CSS);
    x_EKF = StateUpdateAttitude(x_EKF,K_Att,z_CSS);
    P_EKF = CovarianceUpdate(K_Att,P_EKF,R_CSS,H);
end
%---

% 2. Magnetometer
if norm(z_MAG) ~= 0
    H = [0 0 0 0 0 0 1 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 0 0 0 0 0;
         0 0 0 0 0 0 0 0 1 0 0 0 0;
         0 0 0 0 0 0 0 0 0 1 0 0 0];
    K_Att = GainUpdate(H,P_EKF,R_MAG);
    x_EKF = StateUpdateAttitude(x_EKF,K_Att,z_MAG);
    P_EKF = CovarianceUpdate(K_Att,P_EKF,R_MAG,H);
end
%---

% 3. Earth Tracker (position depends on measured accuracy)
for i = 1:n_f
    if norm(z_ET(:,i)) ~= 0
        H_ET = Improved_Numerical_H(x_EKF,catalogue(:,i));
        [K_ET(:,:,i)] = GainUpdate(H_ET,P_EKF,R_ET);
        [x_EKF,y_ET(:,i)] = StateUpdate(x_EKF,K_ET(:,:,i),z_ET(:,i),catalogue(:,i));
        P_EKF = CovarianceUpdate(K_ET(:,:,i),P_EKF,R_ET,H_ET);
        x_EKF(7:10) = quatnormalize(x_EKF(7:10).');
    end
end
%---

% 4. Gyroscope
if norm(z_GYR) ~= 0
    H = [0 0 0 0 0 0 0 0 0 0 1 0 0;
         0 0 0 0 0 0 0 0 0 0 0 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 1];
    K_AngV= GainUpdate(H,P_EKF,R_GYR);
    x_EKF = StateUpdateAngV(x_EKF,K_AngV,z_GYR);
    P_EKF = CovarianceUpdate(K_AngV,P_EKF,R_GYR,H);
end
%---

% 5. GPS (high position accuracy)
if norm(z_GPS) ~= 0
    H = [1 0 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0 0 0 0 0];
    K_Pos = GainUpdate(H,P_EKF,R_GPS);
    x_EKF = StateUpdatePosition(x_EKF,K_Pos,z_GPS);
    P_EKF = CovarianceUpdate(K_Pos,P_EKF,R_GPS,H);
end
%---

% 6. Star Tracker (highest accuracy, final correction)
if norm(z_ST) ~= 0
    H = [0 0 0 0 0 0 1 0 0 0 0 0 0;
         0 0 0 0 0 0 0 1 0 0 0 0 0;
         0 0 0 0 0 0 0 0 1 0 0 0 0;
         0 0 0 0 0 0 0 0 0 1 0 0 0];
    K_Att = GainUpdate(H,P_EKF,R_ST);
    x_EKF = StateUpdateAttitude(x_EKF,K_Att,z_ST);
    P_EKF = CovarianceUpdate(K_Att,P_EKF,R_ST,H);
end
%---

%% Prediction step
xp_EKF = StatePredictionF(x_EKF,I_f,dt_f,Mu_f);
Pp_EKF = CovaraincePrediction(x_EKF,P_EKF,Q_f,dt_f,I_f,Mu_f);
%---

end


%==========================================================================
% Niel Theron
% 19-06-2025
%==========================================================================
% The purpose of this fucntion is to convert from the LLA refrence frame to
% the ECR refrence frame
%==========================================================================
% Input:
% v_L         : LLA Positoin [3x1]  
%       Lat   : Lattitude [deg]
%       Lon   : Longitude [deg]
%       Alt   : Altitude  [km]
% 
% Output:
% v_E         : ECR Refrence frame [3x1] [x, y, z]
%    
%==========================================================================

function v_E = LLA2ECR(v_L)
    v_E = zeros(3,1);
    [v_E(1,1), v_E(2,1), v_E(3,1)] = geodetic2ecef(wgs84Ellipsoid('km'),v_L(1,1),v_L(2,1),v_L(3,1));
end


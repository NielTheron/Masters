function catalogue_eci = Geo2ECI(catalogue_geo, we, t)
%==========================================================================
% Niel Theron
% Convert Geographic Catalogue to ECI Coordinates
%==========================================================================
% Purpose: Convert geographic coordinates (lat/lon) to ECI coordinates
% accounting for Earth rotation at given time
%
% Inputs:
% catalogue_geo - [2 x n_f] matrix of [lat; lon] in degrees
% t            - Current simulation time (seconds from simulation start)
%
% Output:
% catalogue_eci - [3 x n_f] matrix of [x; y; z] in ECI frame (km)
%==========================================================================

% Earth rotation parameters
R_earth = 6378.137;         % WGS84 Earth radius (km)
theta_earth = we * t;       % Earth rotation angle from simulation start

% Number of features
n_f = size(catalogue_geo, 2);

% Initialize output
catalogue_eci = zeros(3, n_f);

% Convert each feature
for i = 1:n_f
    [x,y,z] = geodetic2ecef(wgs84Ellipsoid('km'),catalogue_geo(1, i),catalogue_geo(2, i),0);
    f_ecef = [x;y;z];
    
    % Convert ECEF to ECI (account for Earth rotation)
    R_ecef_to_eci = [cos(theta_earth), -sin(theta_earth), 0;
                     sin(theta_earth),  cos(theta_earth), 0;
                     0,                 0,                1];
    
    catalogue_eci(:, i) = R_ecef_to_eci * f_ecef;
end

end
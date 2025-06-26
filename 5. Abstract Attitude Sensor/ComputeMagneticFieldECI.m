function mag_eci = ComputeMagneticFieldECI(r_ECI, t, we)
% Simplified magnetic field model pointing toward magnetic north pole
% with dipole characteristics

% Convert ECI to ECEF for geographic calculations
theta_earth = we * t;
R_ECI2ECEF = [cos(theta_earth), sin(theta_earth), 0;
             -sin(theta_earth), cos(theta_earth), 0;
              0,                0,               1];

r_ECEF = R_ECI2ECEF * r_ECI * 1000;  % Convert km to meters

% Convert to geodetic coordinates

    [lat_deg, lon_deg, alt_m] = ecef2geodetic(r_ECEF(1), r_ECEF(2), r_ECEF(3), ...
                                              wgs84Ellipsoid(), 'degrees');

lat_rad = deg2rad(lat_deg);

% Simplified dipole magnetic field model
% Magnetic North Pole position (approximate)
mag_north_ECEF = [0; 0; 6378137];  % Simplified: on rotation axis

% Vector from satellite to magnetic north in ECEF
to_mag_north_ECEF = mag_north_ECEF - r_ECEF;
to_mag_north_ECEF = to_mag_north_ECEF / norm(to_mag_north_ECEF);

% Apply magnetic inclination based on latitude
% Inclination angle (simplified model): I = arctan(2*tan(lat))
inclination_rad = atan(2 * tan(lat_rad));

% Local "up" vector (radially outward)
up_ECEF = r_ECEF / norm(r_ECEF);

% Make field tangent to surface, then apply inclination
mag_tangent_ECEF = to_mag_north_ECEF - dot(to_mag_north_ECEF, up_ECEF) * up_ECEF;
mag_tangent_ECEF = mag_tangent_ECEF / norm(mag_tangent_ECEF);

% Apply inclination (rotate toward/away from Earth center)
if abs(inclination_rad) > 1e-6
    % East direction for rotation axis
    east_ECEF = cross(up_ECEF, mag_tangent_ECEF);
    east_ECEF = east_ECEF / norm(east_ECEF);
    
    % Rodrigues rotation formula
    cos_inc = cos(inclination_rad);
    sin_inc = sin(inclination_rad);
    
    mag_field_ECEF = cos_inc * mag_tangent_ECEF + sin_inc * cross(east_ECEF, mag_tangent_ECEF);
else
    mag_field_ECEF = mag_tangent_ECEF;
end

% Convert back to ECI frame
R_ECEF2ECI = R_ECI2ECEF';
mag_eci = R_ECEF2ECI * mag_field_ECEF;
mag_eci = mag_eci / norm(mag_eci);  % Normalize

end
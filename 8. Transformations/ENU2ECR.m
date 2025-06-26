function R = ENU2ECR(lat, lon)


slat = sin(lat); clat = cos(lat);
slon = sin(lon); clon = cos(lon);

% ENU to ECEF matrix
R_ENU2ECR = [-slon,           clon,           0;
              -clon*slat,     -slon*slat,      clat;
               clon*clat,      slon*clat,      slat];



R = R_ENU2ECR;
end
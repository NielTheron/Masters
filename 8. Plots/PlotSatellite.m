function PlotSatellite()

figure;

x = [0 1 1 0];
y = [0 0 1 1];
z = [0 0 0 0]; % flat on Z = 0 plane
fill3(x, y, z, 'red');
axis equal;
grid on;


end
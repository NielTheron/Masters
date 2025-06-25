function PlotCatalogueECI(catalogue_eci,sample)

figure('Name','Catalogue_ECI')

d = size(catalogue_eci,2);

X = zeros(1,d);
Y = zeros(1,d);
Z = zeros(1,d);

quiver3(X,Y,Z,catalogue_eci(2,:,sample),catalogue_eci(1,:,sample),catalogue_eci(3,:,sample),AutoScale="off");

hold on;
    Re = 6378;  % Earth radius in km
    
    % Generate sphere
    [X, Y, Z] = sphere(10);  % Lower resolution for cleaner wireframe
    X = X * Re; Y = Y * Re; Z = Z * Re;
    
    % Create wireframe surface
    mesh(X, Y, Z, 'EdgeColor', 'black', 'FaceColor', 'none', 'LineWidth', 0.5);
    
    axis equal;
    xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
    title('Wireframe Earth Sphere');
    grid on;

end


function PlotETResults(z)

figure;

d = size(z,2);

X = zeros(1,d);
Y = zeros(1,d);
Z = zeros(1,d);

quiver3(X,Y,Z,z(1,:),z(2,:),z(3,:),"AutoScale",'off')


end
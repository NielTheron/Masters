function PlotInnovation(innovation,n_f,dt)

figure('Name','Innovation')
hold on
n = (0:size(innovation,3)-1)*dt;
for i = 1:n_f
    plot(n, squeeze(innovation(1,i,:)));
    plot(n, squeeze(innovation(2,i,:)));
    plot(n, squeeze(innovation(3,i,:)));
end
hold off

end
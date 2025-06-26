function PlotGain(K_est)

d = size(K_est,3);

for i = 1:d
    A = squeeze(K_est(:,1,i,:));
    hold on
    plot(A(1,:))
    plot(A(2,:))
    plot(A(3,:))
    plot(A(4,:))
    plot(A(5,:))
    plot(A(6,:))
    plot(A(7,:))
    plot(A(8,:))
    plot(A(9,:))
    plot(A(10,:))
    plot(A(11,:))
    plot(A(12,:))
    plot(A(13,:))
    hold off
end
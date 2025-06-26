function PlotCovariance(xA,x_est,Pp,n_s)

for i = 1:13
    figure(Name="Varaince plot")
    err = squeeze(sqrt(Pp(i,i,:)))';

    hold on
    plot(xA(i,:))
    plot(x_est(i,:),LineWidth=2)
    t = linspace(1,n_s,n_s);
    x_patch = [t, fliplr(t)];
    y_patch = [x_est(i,:) - err, fliplr(x_est(i,:) + err)];
    patch(x_patch, y_patch, 'b', 'FaceAlpha',0.5, 'EdgeColor','none')
    xlabel('Time');
    ylabel('Estimated Value');
    title('Estimate with Variance');
    grid on
    hold off
end




end
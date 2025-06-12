fig = uifigure('Name','Loading Example');
d = uiprogressdlg(fig,'Title','Loading','Message','Please wait...');

n = 100;
for k = 1:n
    pause(0.01); % Simulate work
    d.Value = k/n;
    d.Message = sprintf('Progress: %d%%', round(100*k/n));
end

close(fig);
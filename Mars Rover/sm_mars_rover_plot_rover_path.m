%% Script to plot the rover path

% Copyright 2021-2022 The MathWorks, Inc

if exist('sm_mars_rover_out') == 0
sm_mars_rover_out = sim("sm_mars_rover");
end

mdlWks = get_param(bdroot,'ModelWorkspace');
roverPath = mdlWks.getVariable('roverPath');
sample_position = mdlWks.getVariable('sample_position');
x = roverPath.x;
y = roverPath.y;
xout = sm_mars_rover_out.logsout{4}.Values.Data;
yout = sm_mars_rover_out.logsout{3}.Values.Data;
refPathx_vec = linspace(min(roverPath.x),max(roverPath.x),length(xout));

figure;
hold on
plot(roverPath.x, roverPath.y,'r--o','LineWidth',1,'DisplayName','Waypoints')
plot(xout,yout,'g-','LineWidth',2,'DisplayName','ActualPath');
plot(sample_position(1),sample_position(2),'bx','MarkerSize',12,'DisplayName','Sample')
title('Rover Path Comparison in XY plane');
legend('Location','Best');
xlabel('X (m)');
ylabel('Y (m)');
movegui('center');
hold off;


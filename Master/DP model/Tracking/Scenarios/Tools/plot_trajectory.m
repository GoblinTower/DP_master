function plot_trajectory(t_c, x_c, y_c, yaw_c, xdot_c, ydot_c)
    
% Plot
fig = figure('DefaultAxesFontSize', 20);
t = tiledlayout(3,2, 'TileSpacing', 'compact');

nexttile;
hold on;
plot(y_c, x_c);
grid();
title('Setpoint path plot');
xlabel('Position east');
ylabel('Position north');
grid on, grid minor;
box on;
xlim('padded');
ylim('padded');
axis equal;
hold off;

nexttile;
hold on;
plot(t_c, y_c);
grid();
title('Setpoint east position');
xlabel('Time [s]');
ylabel('East position [m]');
grid on, grid minor;
box on;
xlim('tight');
ylim('padded');
hold off;

nexttile;
hold on;
plot(t_c, x_c);
grid();
title('Setpoint north position');
xlabel('Time [s]');
ylabel('East position [m]');
grid on, grid minor;
box on;
xlim('tight');
ylim('padded');
hold off;

nexttile;
hold on;
plot(t_c, rad2deg(yaw_c));
grid();
title('Setpoint yaw');
xlabel('Time [s]');
ylabel('Heading [°]');
grid on, grid minor;
box on;
xlim('tight');
ylim('padded');
hold off;

nexttile;
hold on;
plot(t_c, xdot_c);
grid();
title('Setpoint velocity in north direction');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
grid on, grid minor;
box on;
xlim('tight');
ylim('padded');
hold off;

nexttile;
hold on;
plot(t_c, ydot_c);
grid();
title('Setpoint velocity in east direction');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
grid on, grid minor;
box on;
xlim('tight');
ylim('padded');
hold off;

end

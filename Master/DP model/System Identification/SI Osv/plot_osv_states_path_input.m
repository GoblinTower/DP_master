function plot_supply_osv_path_input(t, x, u)
    
    figure(1) % Plotting states

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Velocity and angular momentum in BODY frame %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,3,1);
    plot(t, x(1,:));
    grid();
    title('Velocity in surge direction (u)');
    xlabel('t [s]');
    ylabel('u [m/s]');
    
    subplot(2,3,2);
    plot(t, x(2,:));
    grid();
    title('Velocity in sway direction (v)');
    xlabel('t [s]');
    ylabel('v [m/s]');
    
    subplot(2,3,3);
    plot(t, x(6,:));
    grid();
    title('Angular velocity in yaw (r)');
    xlabel('t [s]');
    ylabel('r [rad/s]');
    
    %%%%%%%%%%%%%%%%%%%%%%%
    %%% Position in NED %%%
    %%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,3,4);
    plot(t, x(7,:));
    grid();
    title('Position x');
    xlabel('t [s]');
    ylabel('x [m]');
    
    subplot(2,3,5);
    plot(t, x(8,:));
    grid();
    title('Position y');
    xlabel('t [s]');
    ylabel('y [m]');
    
    subplot(2,3,6);
    plot(t, x(12,:));
    grid();
    title('Yaw');
    xlabel('t [s]');
    ylabel('Yaw [m]');
    
    figure(2); % Plot depicting the path in the x-y plane
    
    %%%%%%%%%%%%
    %%% Path %%%
    %%%%%%%%%%%%
    hold on
    plot(x(8,:), x(7,:))
    plot(x(8,1), x(7,1), 'go','MarkerSize', 10); % Start position
    plot(x(8,end), x(7,end), 'ro','MarkerSize', 10); % End position
    grid();
    title('Path in x-y plane');
    xlabel('East [m]');
    ylabel('North [m]');
    legend({'Path', 'Start position', "End position"}, 'Location', 'Best');

    figure(3); % Plot depicting input signals
    
    length_time = size(u,2);
    %%%%%%%%%%%%%%%%%%%%%
    %%% Bow thrusters %%%
    %%%%%%%%%%%%%%%%%%%%%
    subplot(3,1,1);
    plot(t(1:length_time), u(1,:))
    grid();
    title('RPS tunnel thruster (1)');
    xlabel('t [s]');
    ylabel('RPM');

    subplot(3,1,2);
    plot(t(1:length_time), u(2,:))
    grid();
    title('RPS azimuth thruster (3)');
    xlabel('t [s]');
    ylabel('RPM');

    subplot(3,1,3);
    plot(t(1:length_time), u(3,:))
    grid();
    title('RPS azimuth thruster (4)');
    xlabel('t [s]');
    ylabel('RPM');

end
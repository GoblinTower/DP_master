function plot_osv_model_lq(t, x, K, u, setpoint, save_plots, folder, file_prefix)
        
    f1 = figure(1); % Plotting states
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
    hold on
    plot(t, x(7,:));
    plot(t, setpoint(1,:));
    grid();
    title('Position north (x)');
    xlabel('t [s]');
    ylabel('North, x [m]');
    legend({'OSV', 'Setpoint'}, 'Location', 'Best');
    hold off

    subplot(2,3,5);
    hold on
    plot(t, x(8,:));
    plot(t, setpoint(2,:));
    grid();
    title('Position east (y)');
    xlabel('t [s]');
    ylabel('East, y [m]');
    legend({'OSV', 'Setpoint'}, 'Location', 'Best');
    hold off
    
    subplot(2,3,6);
    hold on
    plot(t, rad2deg(x(12,:)));
    plot(t, rad2deg(setpoint(3,:)));
    grid();
    title('Vessel heading');
    xlabel('t [s]');
    ylabel('Heading [°]');
    legend({'OSV', 'Setpoint'}, 'Location', 'Best');
    hold off
    
    f2 = figure(2); % Plot depicting the path in the x-y plane
    
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

    f3 = figure(3); % Plot depicting input signals
    
    length_time = size(u,2);
    %%%%%%%%%%%%%%%%%%%%%
    %%% Bow thrusters %%%
    %%%%%%%%%%%%%%%%%%%%%
    subplot(3,1,1);
    plot(t(1:length_time), sign(u(1,:)).*sqrt(abs(u(1,:))))
    grid();
    title('Bow thruster (RPS)');
    xlabel('t [s]');
    ylabel('RPS');

    subplot(3,1,2);
    plot(t(1:length_time), sign(u(2,:)).*sqrt(abs(u(2,:))))
    grid();
    title('Starboard propeller (RPS)');
    xlabel('t [s]');
    ylabel('RPS');

    subplot(3,1,3);
    plot(t(1:length_time), sign(u(3,:)).*sqrt(abs(u(3,:))))
    grid();
    title('Port propeller (RPS)');
    xlabel('t [s]');
    ylabel('RPS');

    f4 = figure(4); % Plot Kalman gain
    
    length_time = size(K,2);
    %%%%%%%%%%%%%%%%%%%
    %%% Kalman gain %%%
    %%%%%%%%%%%%%%%%%%%
    plot(t(1:length_time), K)
    grid();
    title('Kalman gain values');
    xlabel('t [s]');
    ylabel('Gain value');

    if (save_plots)
        save_plot(f1, file_prefix + "_states", folder);
        save_plot(f2, file_prefix + "_path", folder);
        save_plot(f3, file_prefix + "_inputs", folder);
    end

 end

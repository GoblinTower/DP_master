function plot_supply_nmpc_with_disturbance(t, x, x_est, K, u, setpoint, wind, wave, current, save_plots)
    
    f1 = figure(1); % Plotting states

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Velocity and angular momentum in BODY frame %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,3,1);
    plot(t, x(4,:));
    grid();
    title('Velocity in surge direction (u)');
    xlabel('t [s]');
    ylabel('u [m/s]');
    
    subplot(2,3,2);
    plot(t, x(5,:));
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
    
    length_time = size(t,2);
    %%%%%%%%%%%%%%%%%%%%%%%
    %%% Position in NED %%%
    %%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,3,4);
    hold on
    plot(t, x(1,:));
    plot(t, setpoint(1,1:length_time));
    grid();
    title('Position north (x)');
    xlabel('t [s]');
    ylabel('North, x [m]');
    legend({'Supply', 'Setpoint'}, 'Location', 'Best');
    hold off

    subplot(2,3,5);
    hold on
    plot(t, x(2,:));
    plot(t, setpoint(2,1:length_time));
    grid();
    title('Position east (y)');
    xlabel('t [s]');
    ylabel('East, y [m]');
    legend({'Supply', 'Setpoint'}, 'Location', 'Best');
    hold off
    
    subplot(2,3,6);
    hold on
    plot(t, rad2deg(x(3,:)));
    plot(t, rad2deg(setpoint(3,1:length_time)));
    grid();
    title('Vessel heading');
    xlabel('t [s]');
    ylabel('Vessel heading [°]');
    legend({'Supply', 'Setpoint'}, 'Location', 'Best');
    hold off
    
    f2 = figure(2); % Plot depicting the path in the x-y plane
    
    %%%%%%%%%%%%
    %%% Path %%%
    %%%%%%%%%%%%
    hold on
    plot(x(2,:), x(1,:))
    plot(x(2,1), x(1,1), 'go','MarkerSize', 10); % Start position
    plot(x(2,end), x(1,end), 'ro','MarkerSize', 10); % End position
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
    plot(t(1:length_time), u(1,:))
    grid();
    title('Force in surge');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,1,2);
    plot(t(1:length_time), u(2,:))
    grid();
    title('Force in sway');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,1,3);
    plot(t(1:length_time), u(3,:))
    grid();
    title('Momentum in yaw');
    xlabel('t [s]');
    ylabel('Momentum [Nm]');

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

    f5 = figure(5); % Plot external forces
    
    %%%%%%%%%%%%%%%%%%%%%%%
    %%% External forces %%%
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Wind
    wind_length = size(wind,2);
    
    subplot(3,3,1);
    plot(t(1:wind_length), wind(1,:));
    grid();
    title('Wind force (north)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,3,2);
    plot(t(1:wind_length), wind(2,:));
    grid();
    title('Wind force (east)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,3,3);
    plot(t(1:wind_length), wind(3,:));
    grid();
    title('Wind moment (yaw)');
    xlabel('t [s]');
    ylabel('Moment [Nm]');

    % Wave
    wave_length = size(wave,2);
    
    subplot(3,3,4);
    plot(t(1:wave_length), wave(1,:));
    grid();
    title('Wave force (north)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,3,5);
    plot(t(1:wave_length), wave(2,:));
    grid();
    title('Wave force (east)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,3,6);
    plot(t(1:wave_length), wave(3,:));
    grid();
    title('Wave moment (yaw)');
    xlabel('t [s]');
    ylabel('Moment [Nm]');

    % Current
    current_length = size(current,2);
    
    subplot(3,3,7);
    plot(t(1:current_length), current(1,:));
    grid();
    title('Current force (north)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,3,8);
    plot(t(1:current_length), current(2,:));
    grid();
    title('Current force (east)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,3,9);
    plot(t(1:current_length), current(3,:));
    grid();
    title('Current moment (yaw)');
    xlabel('t [s]');
    ylabel('Moment [Nm]');

    f6 = figure(6); % External forces vs. b in state matrix
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% External forces comparison %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    forces_length = min([wind_length, wave_length, current_length]);

    subplot(3,1,1);
    hold on;
    plot(t(1:forces_length), wind(1,1:forces_length) + wave(1,1:forces_length) + ...
        current(1,1:forces_length));
    plot(t(1:forces_length), x_est(7,1:forces_length));
    grid();
    title('External forces (north)');
    xlabel('t [s]');
    ylabel('Force [N]');
    hold off;

    subplot(3,1,2);
    hold on;
    plot(t(1:forces_length), wind(2,1:forces_length) + wave(2,1:forces_length) + ...
        current(2,1:forces_length));
    plot(t(1:forces_length), x_est(8,1:forces_length));
    grid();
    title('External forces (east)');
    xlabel('t [s]');
    ylabel('Force [N]');
    hold off;

    subplot(3,1,3);
    hold on;
    plot(t(1:forces_length), wind(3,1:forces_length) + wave(3,1:forces_length) + ...
        current(3,1:forces_length));
    plot(t(1:forces_length), x_est(9,1:forces_length));
    grid();
    title('Current moment (yaw)');
    xlabel('t [s]');
    ylabel('Moment [Nm]');
    hold off;

    if (save_plots)
        save_plot(f1, "nmp_dist_states", "Results/nmpc_dist/lin_kal");
        save_plot(f2, "nmpc_dist_path", "Results/nmpc_dist/lin_kal");
        save_plot(f3, "nmpc_dist_inputs", "Results/nmpc_dist/lin_kal");
        save_plot(f4, "nmpc_dist_kalman_gains", "Results/nmpc_dist/lin_kal");
        save_plot(f5, "nmpc_dist_external_forces", "Results/nmpc_dist/lin_kal");
        save_plot(f6, "nmpc_dist_external_forces_comparison", "Results/nmpc_dist/lin_kal");
    end

 end

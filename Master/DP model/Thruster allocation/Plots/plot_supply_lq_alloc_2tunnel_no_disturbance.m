function plot_supply_lq_alloc_2_tunnel_no_disturbance(t, x, K, u, rpm, setpoint, save_plots)
    
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
    
    %%%%%%%%%%%%%%%%%%%%%%%
    %%% Position in NED %%%
    %%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,3,4);
    hold on
    plot(t, x(1,:));
    plot(t, setpoint(1,:));
    grid();
    title('Position x');
    xlabel('t [s]');
    ylabel('North, x [m]');
    legend({'Process', 'Setpoint'}, 'Location', 'Best');
    hold off

    subplot(2,3,5);
    hold on
    plot(t, x(2,:));
    plot(t, setpoint(2,:));
    grid();
    title('Position y');
    xlabel('t [s]');
    ylabel('East, y [m]');
    legend({'Process', 'Setpoint'}, 'Location', 'Best');
    hold off
    
    subplot(2,3,6);
    hold on
    plot(t, rad2deg(x(3,:)));
    plot(t, rad2deg(setpoint(3,:)));
    grid();
    title('Yaw');
    xlabel('t [s]');
    ylabel('Yaw [°]');
    legend({'Process', 'Setpoint'}, 'Location', 'Best');
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

    f5 = figure(5); % Plot RPM

    length_time = size(rpm,2);
    %%%%%%%%%%%%%%%%%%%
    %%% Kalman gain %%%
    %%%%%%%%%%%%%%%%%%%
    subplot(2,2,1);
    plot(t(1:length_time), rpm(1,:))
    grid();
    title('RPM main propeller (port)');
    xlabel('t [s]');
    ylabel('RPM');

    subplot(2,2,2);
    plot(t(1:length_time), rpm(2,:))
    grid();
    title('RPM main propeller (starboard)');
    xlabel('t [s]');
    ylabel('RPM');

    subplot(2,2,3);
    plot(t(1:length_time), rpm(3,:))
    grid();
    title('Stern tunnel thruster');
    xlabel('t [s]');
    ylabel('RPM');

    subplot(2,2,4);
    plot(t(1:length_time), rpm(4,:))
    grid();
    title('Bow tunnel thruster');
    xlabel('t [s]');
    ylabel('RPM');

    if (save_plots)
        save_plot(f1, "lq_no_dist_alloc_states", "Results/LQ_no_dist");
        save_plot(f2, "lq_no_dist_alloc_path", "Results/LQ_no_dist");
        save_plot(f3, "lq_no_dist_alloc_inputs", "Results/LQ_no_dist");
        save_plot(f5, "lq_no_dist_alloc_thr", "Results/LQ_no_dist");
    end

 end

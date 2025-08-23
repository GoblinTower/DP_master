function plot_supply_lq_alloc_1_tunnel(t, x, x_est, K, u, wind_abs, wind_beta, wind_force, current_force, wave_force, rpm, f, setpoint, save_plots, folder, file_prefix)
    
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

    f5 = figure(5); % Plot wind
    length_time = size(wind_abs,1);
    %%%%%%%%%%%%
    %%% Wind %%%
    %%%%%%%%%%%%
    subplot(5,1,1);
    plot(t(1:length_time), wind_abs(1:length_time));
    grid();
    title('Wind');
    xlabel('t [s]');
    ylabel('V [m/s]');

    subplot(5,1,2);
    plot(t(1:length_time), rad2deg(wind_beta(1:length_time)));
    grid();
    title('Wind angle');
    xlabel('t [s]');
    ylabel('Degree [°]');

    subplot(5,1,3);
    plot(t(1:length_time), wind_force(1,1:length_time));
    grid();
    title('Force in surge');
    xlabel('t [s]');
    ylabel('F_s [N]');

    subplot(5,1,4);
    plot(t(1:length_time), wind_force(2,1:length_time));
    grid();
    title('Force in sway');
    xlabel('t [s]');
    ylabel('F_s [N]');

    subplot(5,1,5);
    plot(t(1:length_time), wind_force(3,1:length_time));
    grid();
    title('Momentum in yaw');
    xlabel('t [s]');
    ylabel('τ_s [Nm]');

    f6 = figure(6); % Plot current forces and estimated current forces
    length_time = size(current_force,2);
    %%%%%%%%%%%%%%%
    %%% Current %%%
    %%%%%%%%%%%%%%%
    subplot(1,3,1);
    hold on;
    plot(t(1:length_time), current_force(1,1:length_time));
    plot(t(1:length_time), x_est(7,1:length_time));
    grid();
    title('Current force in surge');
    xlabel('t [s]');
    ylabel('F in surge [N]');
    legend({'Current force', 'Est. current force'}, 'Location', 'Best');
    hold off

    subplot(1,3,2);
    hold on;
    plot(t(1:length_time), current_force(2,1:length_time));
    plot(t(1:length_time), x_est(8,1:length_time));
    grid();
    title('Current force in sway');
    xlabel('t [s]');
    ylabel('F in sway [N]')
    legend({'Current force', 'Est. current force'}, 'Location', 'Best');
    hold off

    subplot(1,3,3);
    hold on;
    plot(t(1:length_time), current_force(3,1:length_time));
    plot(t(1:length_time), x_est(9,1:length_time));
    grid();
    title('Current momentum in yaw');
    xlabel('t [s]');
    ylabel('Momentum [Nm]')
    legend({'Current momentum', 'Est. current momentum'}, 'Location', 'Best');
    hold off

    f7 = figure(7); % Plot wave forces
    length_time = size(wave_force,2);
    %%%%%%%%%%%%%%%%%%
    %%% Wave force %%%
    %%%%%%%%%%%%%%%%%%
    subplot(1,3,1);
    plot(t(1:length_time), wave_force(1,1:length_time))
    grid();
    title('Wave force in surge');
    xlabel('t [s]');
    ylabel('F_s [N]');

    subplot(1,3,2);
    plot(t(1:length_time), wave_force(2,1:length_time))
    grid();
    title('Wave force in sway');
    xlabel('t [s]');
    ylabel('F_s [N]');

    subplot(1,3,3);
    plot(t(1:length_time), wave_force(3,1:length_time))
    grid();
    title('Wave momentum in yaw');
    xlabel('t [s]');
    ylabel('τ_s [Nm]');

    f8 = figure(8); % Plot RPM

    length_time = size(rpm,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Setpoint to thrusters %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(3,1,1);
    plot(t(1:length_time), rpm(1,:))
    grid();
    title('RPM main propeller (port)');
    xlabel('t [s]');
    ylabel('RPM');

    subplot(3,1,2);
    plot(t(1:length_time), rpm(2,:))
    grid();
    title('RPM main propeller (starboard)');
    xlabel('t [s]');
    ylabel('RPM');

    subplot(3,1,3);
    plot(t(1:length_time), rpm(3,:))
    grid();
    title('RPM bow thruster');
    xlabel('t [s]');
    ylabel('RPM');

    f9 = figure(9); % Plot RPM

    length_time = size(f,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Force from individual thrusters %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(3,1,1);
    plot(t(1:length_time), f(1,:))
    grid();
    title('Force main propeller (port)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,1,2);
    plot(t(1:length_time), f(2,:))
    grid();
    title('Force main propeller (starboard)');
    xlabel('t [s]');
    ylabel('Force [N]');

    subplot(3,1,3);
    plot(t(1:length_time), f(3,:))
    grid();
    title('Force bow tunnel thruster');
    xlabel('t [s]');
    ylabel('Force [m]');

    if (save_plots)
        save_plot(f1, file_prefix + "states", folder);
        save_plot(f2, file_prefix + "path", folder);
        save_plot(f3, file_prefix + "inputs", folder);
        save_plot(f5, file_prefix + "wind", folder);
        save_plot(f6, file_prefix + "current", folder);
        save_plot(f7, file_prefix + "wave", folder);
        save_plot(f8, file_prefix + "rpm", folder);
        save_plot(f9, file_prefix + "thruster_force", folder);
    end

 end

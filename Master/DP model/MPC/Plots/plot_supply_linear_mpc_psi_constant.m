function plot_supply_linear_mpc_psi_constant(t, x, x_est, K, u, wind_abs, wind_beta, wind_force, current_force, wave_force, setpoint, save_plots)
    
    f1 = figure(1); % Plotting states
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Velocity and angular momentum in BODY frame %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,3,1);
    plot(t, x(4,:));
    grid();
    title('Surge velocity (u)');
    xlabel('t [s]');
    ylabel('u [m/s]');
    
    subplot(2,3,2);
    plot(t, x(5,:));
    grid();
    title('Sway velocity (v)');
    xlabel('t [s]');
    ylabel('v [m/s]');
    
    subplot(2,3,3);
    plot(t, x(6,:));
    grid();
    title('Yaw velocity (r)');
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
    title('Position x');
    xlabel('t [s]');
    ylabel('North, x [m]');
    legend({'Process', 'Setpoint'}, 'Location', 'Best');
    hold off

    subplot(2,3,5);
    hold on
    plot(t, x(2,:));
    plot(t, setpoint(2,1:length_time));
    grid();
    title('Position y');
    xlabel('t [s]');
    ylabel('East, y [m]');
    legend({'Process', 'Setpoint'}, 'Location', 'Best');
    hold off
    
    subplot(2,3,6);
    hold on
    plot(t, rad2deg(x(3,:)));
    plot(t, rad2deg(setpoint(3,1:length_time)));
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

    if (save_plots)
        save_plot(f1, "mpc_states", "Results/MPC_psi_const");
        save_plot(f2, "mpc_path", "Results/MPC_psi_const");
        save_plot(f3, "mpc_inputs", "Results/MPC_psi_const");
        save_plot(f5, "mpc_wind", "Results/MPC_psi_const");
        save_plot(f6, "mpc_current", "Results/MPC_psi_const");
        save_plot(f6, "mpc_wave", "Results/MPC_psi_const");
    end

 end

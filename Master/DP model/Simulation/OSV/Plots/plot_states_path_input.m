function plot_states_and_path_input(t, x, u)
    
    figure(1) % Main 3*3 plot depicting linear/angular velocity and position

    %%%%%%%%%%%%%%%%
    %%% Velocity %%%
    %%%%%%%%%%%%%%%%
    subplot(3,3,1);
    plot(t, x(1,:));
    grid();
    title('Velocity in surge direction (u)');
    xlabel('t [s]');
    ylabel('u [m/s]');
    
    subplot(3,3,2);
    plot(t, x(2,:));
    grid();
    title('Velocity in sway direction (v)');
    xlabel('t [s]');
    ylabel('v [m/s]');
    
    subplot(3,3,3);
    plot(t, x(3,:));
    grid();
    title('Velocity in heave direction (w)');
    xlabel('t [s]');
    ylabel('w [m/s]');
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    %%% Angular velocity %%%
    %%%%%%%%%%%%%%%%%%%%%%%%
    subplot(3,3,4);
    plot(t, x(4,:));
    grid();
    title('Roll (p)');
    xlabel('t [s]');
    ylabel('p [rad/s]');
    
    subplot(3,3,5);
    plot(t, x(5,:));
    grid();
    title('Pitch (q)');
    xlabel('t [s]');
    ylabel('q [rad/s]');
    
    subplot(3,3,6);
    plot(t, x(6,:));
    grid();
    title('Yaw (r)');
    xlabel('t [s]');
    ylabel('r [rad/s]');
    
    %%%%%%%%%%%%%%%%
    %%% Position %%%
    %%%%%%%%%%%%%%%%
    subplot(3,3,7);
    plot(t, x(7,:));
    grid();
    title('Position x');
    xlabel('t [s]');
    ylabel('x [m]');
    
    subplot(3,3,8);
    plot(t, x(8,:));
    grid();
    title('Position y');
    xlabel('t [s]');
    ylabel('y [m]');
    
    subplot(3,3,9);
    plot(t, x(9,:));
    grid();
    title('Position z');
    xlabel('t [s]');
    ylabel('z [m]');
    
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
    xlabel('x [m]');
    ylabel('y [m]');
    legend({'Path', 'Start position', "End position"}, 'Location', 'Best');

    figure(3); % Plot depicting input signals
    
    length_time = size(u,2);
    %%%%%%%%%%%%%%%%%%%%%
    %%% Bow thrusters %%%
    %%%%%%%%%%%%%%%%%%%%%
    subplot(3,2,1);
    plot(t(1:length_time), u(1,:))
    grid();
    title('Control signal bow thruster 1');
    xlabel('t [s]');
    ylabel('RPS [Hz]');

    subplot(3,2,2);
    plot(t(1:length_time), u(2,:))
    grid();
    title('Control signal bow thruster 2');
    xlabel('t [s]');
    ylabel('RPS [Hz]');

    subplot(3,2,3);
    plot(t(1:length_time), u(3,:))
    grid();
    title('Control signal stern azimuth 1');
    xlabel('t [s]');
    ylabel('RPS [Hz]');

    subplot(3,2,4);
    plot(t(1:length_time), u(4,:))
    grid();
    title('Control signal stern azimuth 2');
    xlabel('t [s]');
    ylabel('RPS [Hz]');

    subplot(3,2,5);
    plot(t(1:length_time), rad2deg(u(5,:)))
    grid();
    title('Control signal stern azimuth angle 1');
    xlabel('t [s]');
    ylabel('\alpha [°]');

    subplot(3,2,6);
    plot(t(1:length_time), rad2deg(u(6,:)))
    grid();
    title('Control signal stern azimuth angle 2');
    xlabel('t [s]');
    ylabel('\alpha [°]');

end
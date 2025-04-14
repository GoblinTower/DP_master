function plot_osv_si_comparison(t, u, x, y_si)

    figure % Plot input
    length_time = size(u,2);
    %%%%%%%%%%%%%
    %%% Input %%%
    %%%%%%%%%%%%%
    subplot(3,1,1);
    plot(t(1:length_time), u(1,:))
    grid();
    title('Bow thruster');
    xlabel('t [s]');
    ylabel('RPS');

    subplot(3,1,2);
    plot(t(1:length_time), u(3,:))
    grid();
    title('Main propeller 1');
    xlabel('t [s]');
    ylabel('RPS');

    subplot(3,1,3);
    plot(t(1:length_time), u(4,:))
    grid();
    title('Main propeller 2');
    xlabel('t [s]');
    ylabel('RPS');
    
    figure % Plot comparing velocity in surge, sway and yaw
    %%%%%%%%%%%%
    %%% Path %%%
    %%%%%%%%%%%%
    subplot(1,3,1); 
    hold on;
    plot(t, x(1,:));
    plot(t, y_si(1,:));
    grid();
    title('Velocity in surge direction (u)');
    xlabel('t [s]');
    ylabel('u [m/s]');
    legend({'OSV', 'SI model'}, 'Location', 'Best');
    hold off;
    
    subplot(1,3,2);
    hold on;
    plot(t, x(2,:));
    plot(t, y_si(2,:));
    grid();
    title('Velocity in sway direction (v)');
    xlabel('t [s]');
    ylabel('v [m/s]');
    legend({'OSV', 'SI model'}, 'Location', 'Best');
    hold off;
    
    subplot(1,3,3);
    hold on;
    plot(t, x(6,:));
    plot(t, y_si(3,:));
    grid();
    title('Angular velocity in yaw (r)');
    xlabel('t [s]');
    ylabel('r [rad/s]');
    legend({'OSV', 'SI model'}, 'Location', 'Best');
    hold off;

end
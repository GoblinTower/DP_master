% File containing common setpoint for Monte-Carlo simulations

% Add seed (to ensure same disturbances at every run)
rng(rand_seed, "twister");

%%%%%%%%%%%%%%%%
%%% Setpoint %%%
%%%%%%%%%%%%%%%%
first_distance_change = 10;
second_distance_change = 10;

% First setpoint change (move 10 meter in random direction)
vessel_angle = deg2rad(360*rand());
first_setpoint_change = [first_distance_change*cos(vessel_angle); first_distance_change*sin(vessel_angle); vessel_angle];

% Second setpoint change (move 10 in random direction)
vessel_angle = deg2rad(360*rand());
second_setpoint_change = [first_setpoint_change(1) + second_distance_change*cos(vessel_angle); first_setpoint_change(2) + second_distance_change*sin(vessel_angle); vessel_angle];

% Create setpoint array
setpoint_length = N+horizon_length-1;
setpoint = zeros(3, setpoint_length);
for k=1:setpoint_length
    time = k*dt;
    if (time < 100)
        setpoint(:,k) = [0; 0; 0];                      % Vessel starts in zero position
    elseif (time < 600)
        setpoint(:,k) = first_setpoint_change;           % Vessel goes to first new setpoint change
    else
        setpoint(:,k) = second_setpoint_change;           % Vessel goes to second new setpoint change
    end
end

show_plots_debug = false;
if (show_plots_debug)

    subplot(1,2,1);
    hold on;
    plot(setpoint(2,:), setpoint(1,:));
    plot(setpoint(2,1), setpoint(1,1));
    plot(setpoint(2,end), setpoint(1,end));
    title('Setpoint position');
    xlabel('Position north');
    ylabel('Position east');
    hold off;
    
    subplot(1,2,2);
    plot(rad2deg(setpoint(3,:)));
    title('Heading setpoint');
    xlabel('Time');
    ylabel('Heading (°)');

end
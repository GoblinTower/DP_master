function fig = plot_thrusters(ws, time_array, font_size, length, breadth, scale_forces, scale_surge, scale_sway, scale_momentum, filename, folder)

    % Number of time samples
    number_of_time_samples = size(time_array,2);

    % Fetch number of thrusters
    number_of_thrusters = size(ws.thruster_positions,2);
    
    % Create ship shap
    ship_shape = polyshape([0, breadth/2, breadth/2, -breadth/2, -breadth/2], ...
                           [length/2+length*0.1, length/2, -length/2, -length/2, length/2]);
     
    
    % PLot figure
    fig = figure('DefaultAxesFontSize', font_size);
    t = tiledlayout(number_of_time_samples, 3, "TileSpacing", "compact");
    
    for i=1:number_of_time_samples

        % Get time of snapshot
        time = time_array(i);

        % Vessel heading
        heading = ws.x_array(3,time);
    
        % Rotate vessel
        current_ship = rotate(ship_shape, -rad2deg(heading), [0,0]);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Plot thruster forces %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        nexttile
        hold on
        
        plot(current_ship)
    
        % Looping through thrusters and updating plot
        for i=1:number_of_thrusters

            % Get thruster angles
            thruster_angles = ws.thruster_angles(i);
        
            % Get thruster position in BODY coordinate system
            x_pos = ws.thruster_positions(1,i);
            y_pos = ws.thruster_positions(2,i);
        
            % Calculate thruster position
            thr_length = breadth/20;
            thruster_pos_shape = polyshape([y_pos - thr_length, y_pos, y_pos + thr_length, y_pos], ...
                                           [x_pos, x_pos - thr_length, x_pos, x_pos + thr_length]);
        
            % Rotate thruster position with respect vessel heading
            thruster_pos_shape = rotate(thruster_pos_shape, -rad2deg(heading), [0,0]);
        
            % Plot thruster position
            plot(thruster_pos_shape, 'FaceColor','black');
        
            % Scale thruste force
            thruster_force = ws.f_array(i,time) / scale_forces;
            angle = thruster_angles;
        
            % Only generate force shapes when non-zero force is applied
            if (abs(thruster_force) > 1e-6)
        
                % Calculate force shape
                force_shape = polyshape([y_pos - breadth/20, y_pos + breadth/20, y_pos + breadth/20, y_pos - breadth/20], ...
                                        [x_pos, x_pos, x_pos + thruster_force, x_pos + thruster_force]);
        
                % Rotate force shape with respect to thruster angle
                force_shape = rotate(force_shape, -rad2deg(angle), [y_pos, x_pos]);
        
                % Rotate force with respect vessel heading
                force_shape = rotate(force_shape, -rad2deg(heading), [0,0]);
                
                % Plot thruster force
                plot(force_shape, 'FaceColor','green');
        
            end
        end
        
        title(strcat("Individual thr. forces (t=", num2str(time),")"));
        axis equal;
        box on;
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Plot total thruster forces generated %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        nexttile
        
        % Thruster forces
        thr_force_surge = ws.u_array(1,time)/scale_surge;
        thr_force_sway = ws.u_array(2,time)/scale_sway;
        thr_momentum_yaw = ws.u_array(3,time)/scale_momentum;
        
        plot_forces("Total thr. forces", length, current_ship, breadth, thr_force_surge, thr_force_sway, thr_momentum_yaw, heading, time);
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Plot environmental forces %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        nexttile
        
        % Thruster forces
        env_dist = ws.wind_force_array(:,time) + ws.wave_force_array(:,time) + ws.current_force_array(:,time);
        env_force_surge = env_dist(1)/scale_surge;
        env_force_sway = env_dist(2)/scale_sway;
        env_momentum_yaw = env_dist(3)/scale_momentum;
        
        plot_forces("Total env. forces", length, current_ship, breadth, env_force_surge, env_force_sway, env_momentum_yaw, heading, time);
    
        %%%%%%%%%%%%%%%%%%%%
        %%% Total forces %%%
        %%%%%%%%%%%%%%%%%%%%
        % nexttile
        % 
        % % Thruster forces
        % total_force_surge = thr_force_surge + env_force_surge;
        % total_force_sway = thr_force_sway + env_force_sway;
        % total_momentum_yaw = thr_momentum_yaw + env_momentum_yaw;
        % 
        % plot_forces("Total forces", length, current_ship, breadth, total_force_surge, total_force_sway, total_momentum_yaw, heading, time);
        % 
        % save_plot(fig, filename, folder);

    end
end

function plot_forces(name, length, current_ship, breadth, force_surge, force_sway, momentum_yaw, heading, time)

    hold on
    
    plot(current_ship);           % Always plot ship
    if (force_surge ~= 0)
        force_surge_shape = polyshape([-breadth/20, breadth/20, breadth/20, -breadth/20], ...
                                      [0, 0, force_surge, force_surge]);
        current_force_surge = rotate(force_surge_shape, -rad2deg(heading), [0,0]);
        plot(current_force_surge);
    end
    
    if (force_sway ~= 0)
        force_sway_shape = polyshape([0, 0, force_sway, force_sway], ...
                                     [-breadth/20, breadth/20, breadth/20, -breadth/20]);
        current_force_sway = rotate(force_sway_shape, -rad2deg(heading), [0,0]);
        plot(current_force_sway);
    end
    
    if (momentum_yaw ~= 0)
        force_momentum_shape = polyshape([0, 0, momentum_yaw, momentum_yaw], ...
                                         [length*0.65-breadth/20, length*0.65 + breadth/20, length*0.65 + breadth/20, length*0.65 - breadth/20]);
        current_momentum_shape = rotate(force_momentum_shape, -rad2deg(heading), [0,0]);
        plot(current_momentum_shape);
    end
    
    title(strcat(name, " (t=", num2str(time), ")"));
    axis equal;
    box on;
    hold off

end
classdef AnimateCombined < handle
% Simple class that animates the ship angle and overall forces.
% It also shows the effect of all the thrusters individually.
% It combines the AnimateThrusters class with the AnimateForces class.
% This is done since there are some graphically issues associated with
% running these classes at the same time.
% Furthermore more functionality is added.
%
% The class depicts five plots:
% (1) Depiction of vessel and individually thruster forces.
% (2) Depiction of vessel and overall thruster force/momentum in surge, sway and yaw.
% (3) Depiction of vessel and overall environmental force/momentum in surge, sway and yaw.
% (4) Depiction of vessel and overall total force/momentum in surge, sway and yaw.
% (5) Depiction of lineplots for each thruster force generated.
%

    properties

        ship_figure;                            % Handle to ship figure

        ship_thruster_force_axes;               % Handle to plot with individual thruster forces
        ship_total_thruster_forces_axes;        % Handle to plot with total forces from thusters in surge, sway and yaw
        ship_total_env_forces_axes;             % Handle to plot with total forces from environment in surge, sway and yaw
        ship_total_forces_axes;                 % Handle to plot with total forces in surge, sway and yaw
        
        thruster_force_axes;                    % Handle to lineplot of individual thruster force plot
        thruster_force_handle;                  % Handle to each thruster force animated line

        ship_shape;                             % Ship shape
    
        ship_length;                            % Length of ship
        ship_breadth;                           % Breadth of ship

        thruster_positions;                     % Position of thrusters (each column represent x and y coordinates)
        number_of_thrusters;                    % Numbers of thrusters
        thruster_names;                         % Name of thrusters           
    
    end

    methods

        function obj = AnimateCombined(length, breadth, thruster_positions, thruster_names)
        % This is the constructor to the class.
        % INPUTS:
        % length                : Refers to the length of the vessel.
        % breadth               : Refers to the breadth of the vessel.
        % thruster_position     : Matrix referencing position of thrusters on vessel.
        % thruster names        : Name of each thruster (used in plots).
        %
            
            % Initialized the figure
            % obj.ship_figure = figure('Position', [100, 100, 1200, 1200]);
            obj.ship_figure = figure('units','normalized','outerposition',[0 0 1 1]);

            % Plot of individual thruster forces
            obj.ship_thruster_force_axes = subplot(2,3,1);

            % Plot of thruster forces in surge, sway and yaw
            obj.ship_total_thruster_forces_axes = subplot(2,3,4);

            % Plot of environmental forces in surge, sway and yaw
            obj.ship_total_env_forces_axes = subplot(2,3,5);

            % Plot of total forces in surge, sway and yaw
            obj.ship_total_forces_axes = subplot(2,3,6);

            % Lineplot of individual thruster forces
            obj.thruster_force_axes = subplot(2,3,[2,3]);
            title(obj.thruster_force_axes, "Thruster force values");

            % Create ship shap
            obj.ship_shape = polyshape([0, breadth/2, breadth/2, -breadth/2, -breadth/2], ...
                                       [length/2+length*0.1, length/2, -length/2, -length/2, length/2]);

            % Store ship parameters
            obj.ship_length = length;
            obj.ship_breadth = breadth;

            % Store thruster position
            % Store number of thrusters
            obj.number_of_thrusters = size(thruster_positions,2);
            obj.thruster_positions = thruster_positions;

            % Store thruster names
            obj.thruster_names = thruster_names;

            % Create thruster force handles
            tmp(obj.number_of_thrusters,1) = animatedline;
            for i=1:obj.number_of_thrusters
                tmp(i) = animatedline(obj.thruster_force_axes,'Color', rand(1,3), 'DisplayName', obj.thruster_names(i));
            end
            obj.thruster_force_handle = tmp;

            legend(obj.thruster_force_axes, tmp);
           
        end 
      
        function UpdatePlot(obj, t, heading, thruster_forces, thruster_angles, scale_forces, ...
                thr_force_surge, thr_force_sway, thr_momentum_yaw, env_force_surge, env_force_sway, ...
                env_momentum_yaw, scale_surge, scale_sway, scale_momentum)
        % Function called every iteration of the simulation. This updates
        % all the plots.
        %
        % INPUTS:
        % t                     : Current simulation time.
        % heading               : Vessel heading (in radians).
        % thruster_forces       : Array of thruster forces.
        % thruster_angles       : Array of thruster angles.
        % scale_forces          : Scaling values of individual thruster forces (avoid too long force rectangles in plots). 
        % thr_force_surge       : Thruster force in surge direction.
        % thr_force_sway        : Thruster force in sway direction.
        % thr_momentum_yaw      : Thruster momentum in yaw
        % env_force_surge       : Environmental force in surge direction.
        % env_force_sway        : Environmental force in sway direction.
        % env_momentum_yaw      : Environmental momentum in yaw
        % scale_surge           : Scaling of force in surge direction. 
        % scale_sway            : Scaling of force in sway direction. 
        % scale_momentum        : Scaling of momentum in yaw direction. 
        %

            % Rotate vessel
            current_ship = rotate(obj.ship_shape, -rad2deg(heading), [0,0]);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Force plots in surge, sway and yaw %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Plot thruster forces
            PlottingForcesShip(obj, obj.ship_total_thruster_forces_axes, heading, thr_force_surge, thr_force_sway, thr_momentum_yaw, current_ship, "Forces in surge, sway and yaw due to thrusters", scale_surge, scale_sway, scale_momentum)
            
            % Plot environmental forces
            PlottingForcesShip(obj, obj.ship_total_env_forces_axes, heading, env_force_surge, env_force_sway, env_momentum_yaw, current_ship, "Forces in surge, sway and yaw due to environment", scale_surge, scale_sway, scale_momentum)

            total_force_surge = thr_force_surge + env_force_surge;
            total_force_sway = thr_force_sway + env_force_sway;
            total_momentum_yaw = thr_momentum_yaw + env_momentum_yaw;

            % Plot total forces
            PlottingForcesShip(obj, obj.ship_total_forces_axes, heading, total_force_surge, total_force_sway, total_momentum_yaw, current_ship, "Total forces/momentum in surge, sway and yaw", scale_surge, scale_sway, scale_momentum)
         
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Force plot of individual thruster forces %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            breadth = obj.ship_breadth;

            hold on
            cla(obj.ship_thruster_force_axes);

            plot(obj.ship_thruster_force_axes, current_ship);           % Always plot ship

            title(obj.ship_thruster_force_axes, "Individual thruster forces");

            % Looping through thrusters and updating plot
            for i=1:obj.number_of_thrusters

                % Get thruster position in BODY coordinate system
                x_pos = obj.thruster_positions(1,i);
                y_pos = obj.thruster_positions(2,i);

                % Calculate thruster position
                thr_length = breadth/20;
                thruster_pos_shape = polyshape([y_pos - thr_length, y_pos, y_pos + thr_length, y_pos], ...
                                               [x_pos, x_pos - thr_length, x_pos, x_pos + thr_length]);

                % Rotate thruster position with respect vessel heading
                thruster_pos_shape = rotate(thruster_pos_shape, -rad2deg(heading), [0,0]);

                % Plot thruster position
                plot(obj.ship_thruster_force_axes, thruster_pos_shape, 'FaceColor','black');
    
                % Scale thruste force
                thruster_force = thruster_forces(i) / scale_forces;
                angle = thruster_angles(i);

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
                    plot(obj.ship_thruster_force_axes, force_shape, 'FaceColor','green');

                end
            end

            % Update plot information
            axes(obj.ship_thruster_force_axes)
            axis equal;
            hold off

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Individal thruster force plot %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for i=1:obj.number_of_thrusters
                addpoints(obj.thruster_force_handle(i), t, thruster_forces(i));
            end

        end

        function PlottingForcesShip(obj, plot_axis, heading, force_surge, force_sway, momentum_yaw, current_ship, ...
                title_name, scale_surge, scale_sway, scale_momentum)
        % Helper function for plotting visual containing ship and forces (in surge and sway) and
        % momentum in yaw.
        % 
        % INPUTS:
        % plot_axis             : Handle to the plot under consideration.
        % heading               : Vessel heading (in radians).
        % force_surge           : Force in surge direction.
        % force_sway            : Force in sway direction.
        % momentum_yaw          : Momentum in yaw direction.
        % current_ship          : Reference to ship shape (rotated with respect to the heading angle).
        % title_name            : Title name of plot.
        % scale_surge           : Scaling of force in surge direction. 
        % scale_sway            : Scaling of force in sway direction. 
        % scale_momentum        : Scaling of momentum in yaw direction. 
        %

            % Vessel dimensions
            length = obj.ship_length;                           % Get vessel length
            breadth = obj.ship_breadth;                         % Get vessel breadth

            % Scaling forces and momentum
            force_surge = force_surge/scale_surge;
            force_sway = force_sway/scale_sway;
            momentum_yaw = momentum_yaw/scale_momentum;

            hold on
            cla(plot_axis);

            plot(plot_axis, current_ship);           % Always plot ship
            title(plot_axis, title_name);            % Create plot title

            if (force_surge ~= 0)
                force_surge_shape = polyshape([-breadth/20, breadth/20, breadth/20, -breadth/20], ...
                                              [0, 0, force_surge, force_surge]);
                current_force_surge = rotate(force_surge_shape, -rad2deg(heading), [0,0]);
                plot(plot_axis, current_force_surge);
            end
            
            if (force_sway ~= 0)
                force_sway_shape = polyshape([0, 0, force_sway, force_sway], ...
                                             [-breadth/20, breadth/20, breadth/20, -breadth/20]);
                current_force_sway = rotate(force_sway_shape, -rad2deg(heading), [0,0]);
                plot(plot_axis, current_force_sway);
            end

            if (momentum_yaw ~= 0)
                force_momentum_shape = polyshape([0, 0, momentum_yaw, momentum_yaw], ...
                                                 [length*0.65-breadth/20, length*0.65 + breadth/20, length*0.65 + breadth/20, length*0.65 - breadth/20]);
                current_momentum_shape = rotate(force_momentum_shape, -rad2deg(heading), [0,0]);
                plot(plot_axis, current_momentum_shape);
            end

            axes(plot_axis)
            axis equal;
            hold off

        end
    end
end
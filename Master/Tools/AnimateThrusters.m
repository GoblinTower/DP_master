classdef AnimateThrusters < handle
% Simple plot that animates the ship angle and thruster forces

    properties
        ship_figure;             % Handle to ship figure
        ship_figure_axes;        % Handle to figure axes
        thruster_force_axes;     % Handle thruster force axes
        thruster_force_handle;   % Handle to each thruster force animated line

        ship_shape;              % Ship shape
    
        ship_length;             % Length of ship
        ship_breadth;            % Breadth of ship

        thruster_positions;      % Position of thrusters (each column represent x and y coordinates)
        number_of_thrusters;     % Numbers of thrusters
        thruster_names;          % Name of thrusters

    end
    
    methods

        function obj = AnimateThrusters(length, breadth, thruster_positions, thruster_names)
            
            % Initialized the figure
            obj.ship_figure = figure(300);
            
            % Ship figure
            obj.ship_figure_axes = subplot(1,2,1);

            % Thruster forces
            obj.thruster_force_axes = subplot(1,2,2);
            title(obj.thruster_force_axes, "Thruster force values");
            xlabel(obj.thruster_force_axes, "Time [s]");
            ylabel(obj.thruster_force_axes, "Force [N]");

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

            legend(obj.thruster_force_axes);
           
        end 
      
        function UpdatePlot(obj, t, heading, thruster_forces, thruster_angles, scale_forces)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Ship depiction plot %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            cla(obj.ship_figure_axes);
            length = obj.ship_length; 
            breadth = obj.ship_breadth;

            % Ship shape
            current_ship = rotate(obj.ship_shape, -rad2deg(heading), [0,0]);
       
            hold on
            % Forces and momentum
            plot(obj.ship_figure_axes, current_ship);           % Always plot ship

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
                plot(obj.ship_figure_axes, thruster_pos_shape, 'FaceColor','black');
    
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
                    plot(obj.ship_figure_axes, force_shape, 'FaceColor','green');

                end
            end

            % Update plot information
            title(obj.ship_figure_axes, "Visual depiction of thrusters");
            axes(obj.ship_figure_axes)
            axis equal;
            
            hold off

            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Thruster force plot %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            for i=1:obj.number_of_thrusters
                addpoints(obj.thruster_force_handle(i), t, thruster_forces(i));
            end
        end
    end
end
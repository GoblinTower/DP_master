classdef AnimateForces < handle
% Simple class that animates the ship angle and forces applied to ship

    properties
        ship_figure;             % Handle to ship figure

        ship_figure_axes;        % Handle to plot axes

        ship_shape;              % Ship shape
    
        ship_length;             % Length of ship
        ship_breadth;            % Breadth of ship
    end
    
    methods

        function obj = AnimateForces(length, breadth)
        % This is the constructor to the class.
        % INPUTS:
        % length                : Refers to the length of the vessel.
        % breadth               : Refers to the breadth of the vessel.
        % 

            % Initialized the figure
            obj.ship_figure = figure('DefaultAxesFontSize', 22);
            obj.ship_figure_axes = axes;

            % Create ship shap
            obj.ship_shape = polyshape([0, breadth/2, breadth/2, -breadth/2, -breadth/2], ...
                                       [length/2+length*0.1, length/2, -length/2, -length/2, length/2]);

            % Store ship parameters
            obj.ship_length = length;
            obj.ship_breadth = breadth;
  
        end
      
        function UpdatePlot(obj, heading, force_surge, force_sway, momentum_yaw, scale_surge, scale_sway, scale_yaw)
        % Function called every iteration of the simulation. This updates
        % all the plots.
        %
        % INPUTS:
        % heading               : Vessel heading (in radians).
        % force_surge           : Thruster force in surge.
        % force_sway            : Thruster force in sway.
        % momentum_yaw          : Thruster momentum in yaw.
        % scale_surge           : Scaling of force in surge direction. 
        % scale_sway            : Scaling of force in sway direction. 
        % scale_momentum        : Scaling of momentum in yaw direction. 
        %

            cla(obj.ship_figure_axes);
            length = obj.ship_length; 
            breadth = obj.ship_breadth;

            force_surge = force_surge/scale_surge;
            force_sway = force_sway/scale_sway;
            momentum_yaw = momentum_yaw/scale_yaw;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Force plots in surge, sway and yaw %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Ship shape
            current_ship = rotate(obj.ship_shape, -rad2deg(heading), [0,0]);
       
            hold on
            % Forces and moment
            plot(obj.ship_figure_axes, current_ship);                                           % Always plot ship

            title(obj.ship_figure_axes, "Net forces and moment due to thrusters")               % Plot title

            if (force_surge ~= 0)
                force_surge_shape = polyshape([-breadth/20, breadth/20, breadth/20, -breadth/20], ...
                                              [0, 0, force_surge, force_surge]);
                current_force_surge = rotate(force_surge_shape, -rad2deg(heading), [0,0]);
                plot(obj.ship_figure_axes, current_force_surge);
            end
            
            if (force_sway ~= 0)
                force_sway_shape = polyshape([0, 0, force_sway, force_sway], ...
                                             [-breadth/20, breadth/20, breadth/20, -breadth/20]);
                current_force_sway = rotate(force_sway_shape, -rad2deg(heading), [0,0]);
                plot(obj.ship_figure_axes, current_force_sway);
            end

            if (momentum_yaw ~= 0)
                force_momentum_shape = polyshape([0, 0, momentum_yaw, momentum_yaw], ...
                                                 [length*0.65-breadth/20, length*0.65 + breadth/20, length*0.65 + breadth/20, length*0.65 - breadth/20]);
                current_momentum_shape = rotate(force_momentum_shape, -rad2deg(heading), [0,0]);
                plot(obj.ship_figure_axes, current_momentum_shape);
            end

            axes(obj.ship_figure_axes)
            axis equal;

            hold off

            drawnow;
        end

        function delete(obj)
            close obj.ship_figure;
        end
    end
end
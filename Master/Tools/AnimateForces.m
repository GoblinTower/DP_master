classdef AnimateForces < handle
% Simple plot that animates the ship angle and forces applied to ship

    properties
        ship_figure;             % Handle to ship figure
        ship_figure_axes;        % Handle to figure axes
        ship_shape;              % Ship shape
    
        ship_length;             % Length of ship
        ship_breadth;            % Breadth of ship
    end
    
    methods

        function obj = AnimateForces(length, breadth)
            
            % Initialized the figure
            obj.ship_figure = figure;
            obj.ship_figure_axes = axes;

            % Create ship shap
            obj.ship_shape = polyshape([0, breadth/2, breadth/2, -breadth/2, -breadth/2], ...
                                       [length/2+length*0.1, length/2, -length/2, -length/2, length/2]);

            % Store ship parameters
            obj.ship_length = length;
            obj.ship_breadth = breadth;
  
        end
      
        function UpdatePlot(obj, heading, force_north, force_east, momentum_yaw, scale_north, scale_east, scale_yaw)

            cla(obj.ship_figure, 'reset');
            length = obj.ship_length; 
            breadth = obj.ship_breadth;

            force_north = force_north/scale_north;
            force_east = force_east/scale_east;
            momentum_yaw = momentum_yaw/scale_yaw;
            
            % Ship shape
            current_ship = rotate(obj.ship_shape, -rad2deg(heading), [0,0]);
       
            hold on
            % Forces and momentum
            plot(obj.ship_figure_axes, current_ship);           % Always plot ship

            if (force_north ~= 0)
                force_north_shape = polyshape([-breadth/20, breadth/20, breadth/20, -breadth/20], ...
                                              [0, 0, force_north, force_north]);
                current_force_north = rotate(force_north_shape, -rad2deg(heading), [0,0]);
                plot(obj.ship_figure_axes, current_force_north);
            end
            
            if (force_east ~= 0)
                force_east_shape = polyshape([0, 0, force_east, force_east], ...
                                             [-breadth/20, breadth/20, breadth/20, -breadth/20]);
                current_force_east = rotate(force_east_shape, -rad2deg(heading), [0,0]);
                plot(obj.ship_figure_axes, current_force_east);
            end

            if (momentum_yaw ~= 0)
                force_momentum_shape = polyshape([0, 0, momentum_yaw, momentum_yaw], ...
                                                 [length*0.65-breadth/20, length*0.65 + breadth/20, length*0.65 + breadth/20, length*0.65 - breadth/20]);
                current_momentum_shape = rotate(force_momentum_shape, -rad2deg(heading), [0,0]);
                plot(obj.ship_figure_axes, current_momentum_shape);
            end
            hold off

            axis equal;

            % Plot bar plot
            % name = ['Force surge', 'Force sway', 'Momentum east'];
            % values = [force_north, force_east, momentum_yaw];
            % bar(obj.ship_bar_axes, name, values);

        end
    end
end
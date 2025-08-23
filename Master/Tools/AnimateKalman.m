classdef AnimateKalman < handle
% Simple plot that animates the ship position and vessel heading.
% Depicting both the measured position/heading and estimated
% position/heading together with the setpoint.

    properties
        plot_estimated_position_handle        % Line handle to estimated position
        plot_measurement_position_handle      % Line handle to measured position
        plot_setpoint_position_handle         % Line handle to plot position setpoint

        plot_estimated_psi_handle             % Line handle to plot estimated vessel heading
        plot_measured_psi_handle              % Line handle to plot measured vessel heading
        plot_setpoint_psi_handle              % Line handle to plot vessel heading setpoint
    end
    
    methods

        function obj = AnimateKalman()
            % Initialized the figure
            figure(100);

            % Position plot
            subplot(1,2,1);
            title('Ship position (north, east)');
            xlabel('East position [m]');
            ylabel('North position [m]');
            legend();

            obj.plot_estimated_position_handle = animatedline('Color','b',...
                'DisplayName','Estimated position');
            obj.plot_measurement_position_handle = animatedline('Marker','x','Color','k',...
                'DisplayName','Measured position', 'LineStyle','none');
            obj.plot_setpoint_position_handle = animatedline('Marker','o','Color','g',...
                'MarkerSize',8,'DisplayName','Setpoint','MarkerFace','g','LineStyle','none');
            
            % Vessel heading plot
            subplot(1,2,2);
            title('Vessel heading [°]');
            xlabel('Time');
            ylabel('Vessel heading [°]');
            legend();

            obj.plot_estimated_psi_handle = animatedline('Color','b',...
                'DisplayName','Estimated heading');
            obj.plot_measured_psi_handle = animatedline('Marker','x','Color','k',...
                'DisplayName','Measured heading','LineStyle','none');
            obj.plot_setpoint_psi_handle = animatedline('Marker','o','Color','g',...
                'MarkerSize',8,'DisplayName','Heading setpoint','MarkerFace','g','LineStyle','none');
            
            drawnow;
            
        end
      
        function UpdatePlot(obj, t, x_north, x_east, x_psi, y_north, y_east, y_psi,...
                setpoint_north, setpoint_east, setpoint_psi)
            % This function updates plot
            %
            % INPUT:
            % t                 : Current time
            % x_north           : Estimated north position
            % x_east            : Estimated east position
            % x_psi             : Estimated vessel heading
            % y_north           : Measurement of north position
            % y_east            : Measurement of east position
            % y_psi             : Measurement of vessel heading
            % setpoint_north    : Setpoint of north position
            % setpoint_east     : Setpoint of east position
            % setpoint_psi      : Setpoint of vessel heading
            %

            % Position
            addpoints(obj.plot_estimated_position_handle, x_east, x_north);
            addpoints(obj.plot_measurement_position_handle, y_east, y_north);
                
            clearpoints(obj.plot_setpoint_position_handle);
            addpoints(obj.plot_setpoint_position_handle, setpoint_east, setpoint_north);

            % Vessel heading
            addpoints(obj.plot_estimated_psi_handle, t,  rad2deg(x_psi));
            addpoints(obj.plot_measured_psi_handle, t, rad2deg(y_psi));
            
            clearpoints(obj.plot_setpoint_psi_handle);
            addpoints(obj.plot_setpoint_psi_handle, t, rad2deg(setpoint_psi));
            
            drawnow;
           
        end
    end
end
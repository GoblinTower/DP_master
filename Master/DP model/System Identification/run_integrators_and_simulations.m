% Script that runs system identification and control based integrator

% Clean up workspace
clear, clc, close all;

run_integrator_identification = true;
run_simulations = false;
run_error_simulation = false;

external_scenario = 'integrator_methods';

%%%%%%%%%%%%%%%%%%%%
%%% Supply model %%%
%%%%%%%%%%%%%%%%%%%%
if (run_integrator_identification)
    %% SI Supply model
    for mode=1:4
        clearvars -except si_methods external_scenario mode run_integrator_identification run_simulations run_error_simulation                               % First remove previous history
        clc;                                                                                                                                                 % Clear command prompt and close figures
        disp(strcat(['Running integrator mode ', num2str(mode), ' on supply model']));
        run 'Integrator Supply\collect_model_data_supply';
    end
end

%% Run SI Supply model simulations
if (run_simulations)
    % Run with disturbance
    run_with_disturbance = true;
    for mode=2:3
        clearvars -except si_methods number_of_methods external_scenario mode run_with_disturbance run_integrator_identification run_simulations run_error_simulation                        % First remove previous history
        clc;                                                                                                                                                                                 % Clear command prompt and close figures
        disp(strcat(['Running control on model identified using supply model in mode ', num2str(mode), ' with external disturbance']));
        sysid = strcat('int_mode', num2str(mode));
        load_path = strcat(['Log\integrator_ssm_supply_mode', num2str(mode)]);
        run 'LQ_control_supply_integrator_design';
    end
end

% if (run_error_simulation)
%     mode1 = load('Log\integrator_ssm_supply_mode1.mat');
%     data1 = load('Integrator Supply\Log\integrator_supply_data');
%     y_array = [data1.x_array(4:6,1:end-1)];
%     u_array = [data1.u_array(1:3,:)];
%     data = iddata(y_array', u_array', mode1.dt);
%     sys = ss(mode1.A, mode1.B, mode1.C, zeros(3,3), mode1.dt);
% 
%     [ymod,fit,ic] = compare(data, sys);
% end

clear, clc, close all;
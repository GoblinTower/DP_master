% Script that runs system identification and control based on different
% vessel models. Intended for saving time and ensuring consistency.

% Clean up workspace
clear, clc, close all;

si_methods = {'dsr', 'dsr_e', 'pem'};
number_of_methods = size(si_methods,2);

external_scenario = 'si_methods';

run_si = false;

%%%%%%%%%%%%%%%%%%%%
%%% Supply model %%%
%%%%%%%%%%%%%%%%%%%%

% SI Supply model
if (run_si)
    for i=1:number_of_methods
        clearvars -except si_methods number_of_methods external_scenario i run_si    % First remove previous history
        clc, close all;                                                              % Clear command prompt and close figures
        disp(strcat("Running ", si_methods(i), " on supply model"));
        sysid = cell2mat(si_methods(i));
        run 'SI Supply\collect_model_data_supply';
    end
end

%% Run SI Supply model simulations

% Run with disturbance
run_with_disturbance = true;
for i=1:number_of_methods
    clearvars -except si_methods number_of_methods external_scenario i run_with_disturbance run_si                  % First remove previous history
    clc, close all;                                                                                                 % Clear command prompt and close figures
    disp(strcat("Running control on model identified using ", si_methods(i), " with external disturbance"));
    sysid = cell2mat(si_methods(i));
    run 'LQ_control_supply';
end

% Run without disturbance
run_with_disturbance = false;
for i=1:number_of_methods
    clearvars -except si_methods number_of_methods external_scenario i run_with_disturbance run_si                  % First remove previous history
    clc, close all;                                                                                                 % Clear command prompt and close figures
    disp(strcat("Running control on model identified using ", si_methods(i), " without external disturbance"));
    sysid = cell2mat(si_methods(i));
    run 'LQ_control_supply';
end

%%%%%%%%%%%%%%%%
%% OSV model %%%
%%%%%%%%%%%%%%%%

%% SI OSV model
if (run_si)
    for i=1:number_of_methods
        clearvars -except si_methods number_of_methods external_scenario i run_si    % First remove previous history
        clc, close all;                                                              % Clear command prompt and close figures
        disp(strcat("Running ", si_methods(i), " on OSV model"));
        sysid = cell2mat(si_methods(i));
        run 'SI OSV\collect_model_data_osv';
    end
end

%% Run SI OSV model simulations

% Run with disturbance
run_with_disturbance = true;
for i=1:number_of_methods
    clearvars -except si_methods number_of_methods external_scenario i run_with_disturbance run_si                  % First remove previous history
    clc, close all;                                                                                                 % Clear command prompt and close figures
    disp(strcat("Running control on model identified using ", si_methods(i), " with external disturbance"));
    sysid = cell2mat(si_methods(i));
    run 'LQ_optimal_control_osv';
end

% Run without disturbance
run_with_disturbance = false;
for i=1:number_of_methods
    clearvars -except si_methods number_of_methods external_scenario i run_with_disturbance run_si                  % First remove previous history
    clc, close all;                                                                                                 % Clear command prompt and close figures
    disp(strcat("Running control on model identified using ", si_methods(i), " without external disturbance"));
    sysid = cell2mat(si_methods(i));
    run 'LQ_optimal_control_osv';
end

%%%%%%%%%%%%%%%%%%%%%
%%% Balchen model %%%
%%%%%%%%%%%%%%%%%%%%%

%% SI Balchen model
if (run_si)
    for i=1:number_of_methods
        clearvars -except si_methods number_of_methods external_scenario i run_si    % First remove previous history
        clc, close all;                                                              % Clear command prompt and close figures
        disp(strcat("Running ", si_methods(i), " on Balchen model"));
        sysid = cell2mat(si_methods(i));
        run 'SI Balchen\collect_model_data_balchen';
    end
end

%% Run SI Balchen model simulations

% Run with disturbance
run_with_disturbance = true;
for i=1:number_of_methods
    clearvars -except si_methods number_of_methods external_scenario i run_with_disturbance run_si                        % First remove previous history
    clc, close all;                                                                                                       % Clear command prompt and close figures
    disp(strcat("Running control on model identified using ", si_methods(i), " with external disturbance"));
    sysid = cell2mat(si_methods(i));
    run 'LQ_optimal_control_balchen';
end

% Run without disturbance
run_with_disturbance = false;
for i=1:number_of_methods
    clearvars -except si_methods number_of_methods external_scenario i run_with_disturbance run_si                  % First remove previous history
    clc, close all;                                                                                                 % Clear command prompt and close figures
    disp(strcat("Running control on model identified using ", si_methods(i), " without external disturbance"));
    sysid = cell2mat(si_methods(i));
    run 'LQ_optimal_control_balchen';
end

clear, clc, close all;

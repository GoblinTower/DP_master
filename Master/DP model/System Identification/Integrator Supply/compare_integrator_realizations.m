% Generate SI realization versus data
clear, clc, close all;

load_path = strcat('Log\integrator_supply_data');
data = load(load_path);
y_array = [data.x_array(4:6,1:end-1)];
u_array = [data.u_array(1:3,:)];
t_array = data.t_array;

% Load data
load_path = strcat('..\Log\integrator_ssm_supply_mode1');
model_mode1 = load(load_path);
y_array_mode1 = compare_against_data(model_mode1, y_array, u_array);

load_path = strcat('..\Log\integrator_ssm_supply_mode2');
model_mode2 = load(load_path);
y_array_mode2 = compare_against_data(model_mode2, y_array, u_array);

load_path = strcat('..\Log\integrator_ssm_supply_mode3');
model_mode3 = load(load_path);
y_array_mode3 = compare_against_data(model_mode3, y_array, u_array);

load_path = strcat('..\Log\integrator_ssm_supply_mode4');
model_mode4 = load(load_path);
y_array_mode4 = compare_against_data(model_mode3, y_array, u_array);

% Mode 1: Run with only as B unknown, A = C = I
% Mode 2: Run with B and C as unknowns, A = I
% Mode 3: Run with A, B and C as unknown
% Mode 4: Run with A and B unkown, A is upper triangular with ones on the
% diagonal
y_data_array = {y_array_mode1, y_array_mode4, y_array_mode2};
model_case = {'(strategy 1)', '(strategy 2)', '(strategy 3)'};
titles = {'Surge velocity ', 'Sway velocity ', 'Angular velocity '};

for i=1:3

    % Plot
    fig = figure('DefaultAxesFontSize', 20);
    t = tiledlayout(3,1, "TileSpacing", "compact");

    y_data_estimated = cell2mat(y_data_array(i));
    length = min(size(t_array,2), size(y_data_estimated,2));
    
    title_name = strcat([cell2mat(titles(1)), cell2mat(model_case(i))]);
    nexttile;
    hold on;
    plot(t_array(1:length), y_array(1,1:length));
    plot(t_array(1:length), y_data_estimated(1,1:length));
    grid();
    title(title_name);
    xlabel('Time [s]');
    ylabel('u [m/s]');
    legend({'Vessel data', 'SI model'}, 'Location', 'northeast');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;
    
    title_name = strcat([cell2mat(titles(2)), cell2mat(model_case(i))]);
    nexttile;
    hold on;
    plot(t_array(1:length), y_array(2,1:length));
    plot(t_array(1:length), y_data_estimated(2,1:length));
    grid();
    title(title_name);
    xlabel('Time [s]');
    ylabel('v [m/s]');
    legend({'Vessel data', 'SI model'}, 'Location', 'northeast');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;
    
    title_name = strcat([cell2mat(titles(3)), cell2mat(model_case(i))]);
    nexttile;
    hold on;
    plot(t_array(1:length), y_array(3,1:length));
    plot(t_array(1:length), y_data_estimated(3,1:length));
    grid();
    title(title_name);
    xlabel('Time [s]');
    ylabel('r [rad/s]');
    legend({'Vessel data', 'SI model'}, 'Location', 'northeast');
    grid on, grid minor;
    box on;
    ylim('padded');
    hold off;

    save_plot(fig, strcat('compare_case', num2str(i)), 'Plots');

    % clf(fig);

end

function y_estimated = compare_against_data(model, y_array, u_array)
    
    % Fetch model matrices
    A = model.A;
    B = model.B;
    C = model.C;
   
    dt = model.dt;              % Get timestep
    x0 = [0; 0; 0];             % Initial state
    N = size(y_array, 2);       % Number of timesteps
    y_estimated = zeros(3,N);   % Preallocate estimated y array

    % Simulation
    x = x0;
    for i=1:N

        y_estimated(:,i) = C*x;

        x = A*x + B*u_array(:,i);
    end

end
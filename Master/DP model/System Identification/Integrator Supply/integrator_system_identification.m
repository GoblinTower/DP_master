% Identifies underlying model assuming integrator
run 'setup_supply_variable_input_prbs';

% Compare output vs model output
debug_output = true;

% Fetch logged data from file
load_path = strcat('Log\integrator_supply_data');
data = load(load_path);

% x_array
% x[1:3]: x, y, psi (position in north, east and vessel heading)
% x[4:6]: u, v, r (velocity in surge, sway and yaw)
% We want to identify the surge, sway and yaw rotation dynamics
y_array = [data.x_array(4:6,1:end-1)];
u_array = [data.u_array(1:3,:)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Integrator System identification %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Using multiple starting points to improve estimate
rng(12,"twister");
fval_min = inf;

W = diag([1,1,2]);

% Choose mode
% Mode 1: Run with only as B unknown, A = C = I
% Mode 2: Run with B and C as unknowns, A = I
% Mode 3: Run with A, B and C as unknown
% Mode 4: Run with A and B unkown, A is upper triangular with ones on
% diagonal
if (~exist('external_scenario', 'var'))
    mode = 4; % Manual mode
end

if (mode == 1)
    % Run with only as B unknown, A = C = I
    n = 500;
    for i=1:n 
        
        % Starting point
        b0 = 1e-8*2*(rand(1,9)-0.5);
    
        % Optimization
        [b_sol, fval] = fmincon(@(b) integrator_identification_objective_function_mode1(b, y_array, u_array, W), b0, [], [], [], [], [], [], [], options);
    
        if (fval < fval_min)
            fval_min = fval;
            b_min = b_sol;
        end
    end
    
    % SSM
    A = eye(3);
    B = reshape(b_min,[3,3])';
    C = eye(3);
    D = zeros(3,3);

    % Save data
    save_path = strcat('..\Log\integrator_ssm_supply_mode1');
    save(save_path, 'dt', 'A', 'B', 'C');

elseif (mode == 2)
    % Mode 2: Run with B and C as unknowns, A = I
    n = 10;
    for i=1:n

        progress = strcat(['Progress: ' num2str(i), '/', num2str(n)]);
        disp(progress);

        for j=1:n
            for k=1:n

                % Starting point
                f0 = [10^(0.5-j)*2*(rand(1,9)-0.5), 10^(0.5-k)*2*(rand(1,9)-0.5)];

                % Optimization
                [f_sol, fval] = fmincon(@(f) integrator_identification_objective_function_mode2(f, y_array, u_array, W), f0, [], [], [], [], [], [], [], options);

                if (fval < fval_min)
                    fval_min = fval;
                    f_min = f_sol;
                end
            end
        end
    end
        
    % n = 500;
    % for i=1:n 
    % 
    %     % Starting point
    %     f0 = [1e-8*2*(rand(1,9)-0.5), 1e0*2*(rand(1,9)-0.5)];
    % 
    %     % Optimization
    %     [f_sol, fval] = fmincon(@(f) integrator_identification_objective_function_mode2(f, y_array, u_array, W), f0, [], [], [], [], [], [], [], options);
    % 
    %     if (fval < fval_min)
    %         fval_min = fval;
    %         f_min = f_sol;
    %     end
    % end
    
    % SSM
    A = eye(3);
    B = reshape(f_min(1:9),[3,3])';
    C = reshape(f_min(10:18),[3,3])';
    D = zeros(3,3);

    % Save data
    save_path = strcat('..\Log\integrator_ssm_supply_mode2');
    save(save_path, 'dt', 'A', 'B', 'C');

elseif (mode == 3)
    % Mode 3: Run with A, B and C as unknown
    n = 10;
    for i=1:n 
        progress = strcat(['Progress: ' num2str(i), '/', num2str(n)]);
        disp(progress);
        for j=1:n
            for k =1:n
            
                % Starting point
                f0 = [10^(0.5-i)*2*(rand(1,9)-0.5), 10^(0.5-j)*2*(rand(1,9)-0.5), 10^(0.5-k)*2*(rand(1,9)-0.5)];
            
                % Optimization
                [f_sol, fval] = fmincon(@(f) integrator_identification_objective_function_mode3(f, y_array, u_array, W), f0, [], [], [], [], [], [], [], options);
            
                if (fval < fval_min)
                    fval_min = fval;
                    f_min = f_sol;
                end
            end
        end
    end
    
    % SSM
    A = reshape(f_min(1:9),[3,3])';
    B = reshape(f_min(10:18),[3,3])';
    C = reshape(f_min(19:27),[3,3])';
    D = zeros(3,3);

    % Save data
    save_path = strcat('..\Log\integrator_ssm_supply_mode3');
    save(save_path, 'dt', 'A', 'B', 'C');

elseif(mode == 4)
    % Run with only A and B as unknown. A is here an upper triangular
    % matrix with ones on the diagonal.
    n = 10;
    for i=1:n

        progress = strcat(['Progress: ' num2str(i), '/', num2str(n)]);
        disp(progress);

        for j=1:n
            for k=1:n

                % Starting point
                f0 = [10^(0.5-j)*2*(rand(1,9)-0.5), 10^(0.5-k)*2*(rand(1,3)-0.5)];

                % Optimization
                [f_sol, fval] = fmincon(@(f) integrator_identification_objective_function_mode4(f, y_array, u_array, W), f0, [], [], [], [], [], [], [], options);

                if (fval < fval_min)
                    fval_min = fval;
                    f_min = f_sol;
                end
            end
        end
    end

    % fun = @(b) integrator_identification_objective_function_mode1_extended(b, y_array, u_array, W);
    % problem = createOptimProblem('fmincon', 'objective', fun, ...
    %     'x0', 0.1*rand(1,12), 'lb', -1*ones(1,12), 'ub', 5*ones(1,12), 'options', options);
    % 
    % ms = MultiStart;
    % [b_min, fval_min] = run(ms, problem, n);
    
    % SSM
    A = eye(3);
    A(1,2) = f_min(10);
    A(1,3) = f_min(11);
    A(2,3) = f_min(12);
    B = reshape(f_min(1:9),[3,3])';
    C = eye(3);
    D = zeros(3,3);

    % Save data
    save_path = strcat('..\Log\integrator_ssm_supply_mode4');
    save(save_path, 'dt', 'A', 'B', 'C');
end

if (debug_output)
    data = iddata(y_array', u_array', dt);
    sys = ss(A, B, C, D, dt);
    
    fig = figure;
    compare(data, sys);
    save_plot(fig, strcat(['integrator_mode_compare', num2str(mode)]), '..\Workspace\Integrator\');
end


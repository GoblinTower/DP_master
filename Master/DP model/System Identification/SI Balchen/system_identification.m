% Script for performing system identification of Balchen model data
addpath("..\..\..\..\Libraries\DSR");

% Compare output vs model output
debug_output = true;

% Fetch logged data from file
load_path = strcat('Log\', sysid, '_balchen_data');
data = load(load_path);

% x_array
% x(1)               : North position (x)
% x(2)               : East position (y)
% x(3)               : Vessel heading relative to north (psi)
% x(4)               : Distance travelled in surge direction (x_su)
% x(5)               : Distance travelled in sway direction (y_su)
% x(6)               : Velocity in surge (u)
% x(7)               : Velocity in sway (v)
% x(8)               : Vessel heading angular velocity (r)
% We want to identify the surge, sway and yaw rotation dynamics
y_array = [data.x_array(6:8,1:end-1)];
u_array = [data.u_array(1:3,:)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% System identification - DSR %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (strcmp(sysid,'dsr'))

    % System identification parameters (DSR)
    L_sys = 3;              % Number of block rows in extended observability matrix
    n_sys = n_si_dim;       % Number of states
    g_sys = 0;              % Structure parameter
    J_sys = L_sys;          % Past horizon parameter
    M_sys = 1;
    
    [A,B,C,D,CF,F,x0] = dsr(y_array',u_array',L_sys,g_sys,J_sys,M_sys,n_sys);
    
    % Save data for DSR
    save_path = strcat('..\Log\', sysid, '_ssm_balchen');
    save(save_path, 'dt', 'A', 'B', 'C');

    if (debug_output)
        data = iddata(y_array', u_array', dt);
        sys = ss(A, B, C, D, dt);
        
        figure;
        compare(data, sys);
    end

elseif (strcmp(sysid,'dsr_e'))

    % System identification parameters (DSR_e)
    L_sys = 3;              % Number of block rows in extended observability matrix
    g_sys = 0;              % Structure parameter
    J_sys = L_sys;          % Past horizon parameter
    n_sys = n_si_dim;       % Number of states
    
    [A,B,C,D,K,F,x0,Ef1,Yp] = dsr_e(y_array',u_array',L_sys,g_sys,J_sys,n_sys);

    % Save data for DSR_e
    save_path = strcat('..\Log\', sysid, '_ssm_balchen');
    save(save_path, 'dt', 'A', 'B', 'C');

    if (debug_output)
        data = iddata(y_array', u_array', dt);
        sys = ss(A, B, C, D, dt);
    
        figure;
        compare(data, sys);
    end

elseif (strcmp(sysid,'pem'))
    data = iddata(y_array', u_array', dt);
    sys = pem(data, n_si_dim);

    % Save data for PEM
    save_path = strcat('..\Log\', sysid, '_ssm_balchen');
    A = sys.A; 
    B = sys.B; 
    C = sys.C;
    save(save_path, 'dt', 'A', 'B', 'C');

    if (debug_output)
        figure;
        compare(data, sys);
    end

end

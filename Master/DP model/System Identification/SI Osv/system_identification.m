% Script for performing system identification of OSV data
addpath("..\..\..\..\Libraries\DSR");

% Compare output vs model output
debug_output = true;

% Fetch logged data from file
load_path = strcat('Log\', sysid, '_osv_data');
data = load(load_path);

% x_array
% x[1:3]: u, v, w (velocity in surge, sway and heave)
% x[4:6]: p, q, r (angular velocity in roll, pitch and yaw)
% x[7:9]: x, y, z (position, relative to start position)
% x[10:12]: phi, theta, psi: (angular position in roll, pitch and yaw)
% Assume position (GPS) and velocity is known
y_array = [data.x_array(1:2,1:end-1); data.x_array(6,1:end-1)];
% Note that second propeller thruster is not included here (set to be inactive
% during the simulation, i.e. zero control input).
u_array= [data.u_squared(1,:); data.u_squared(3:4,:)]; % degree2rad(azimuth_angle_1),deg2rad(azimuth_angle_2);
% u_array_dsr = [data.u_array(1,:); data.u_array(3:4,:)]; % degree2rad(azimuth_angle_1),deg2rad(azimuth_angle_2);

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
    save_path = strcat('..\Log\', sysid, '_ssm_osv');
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
    save_path = strcat('..\Log\', sysid, '_ssm_osv');
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
    save_path = strcat('..\Log\', sysid, '_ssm_osv');
    A = sys.A; 
    B = sys.B; 
    C = sys.C;
    save(save_path, 'dt', 'A', 'B', 'C');

    if (debug_output)
        figure;
        compare(data, sys);
    end

end
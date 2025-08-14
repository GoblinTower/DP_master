% Simple script for plotting and comparing system identification algoritms
% in the context of LQ optimal control
clear, clc, close all;

% Load data
dsr_data = load('Results\LQ_supply\dsr_no_distdata.mat');
dsr_e_data = load('Results\LQ_supply\dsr_e_no_distdata.mat');
pem_data = load('Results\LQ_supply\pem_no_distdata.mat');

% Compute IAE (Integrated Absolute Error)
disp("Integrated Absolute Error (IAE):")
iae_dsr = sum(abs(dsr_data.setpoint - dsr_data.x_array(1:3,:)),2)'
iae_dsr_e = sum(abs(dsr_e_data.setpoint - dsr_e_data.x_array(1:3,:)),2)'
iae_pem = sum(abs(pem_data.setpoint - pem_data.x_array(1:3,:)),2)'

% Compute TV (Total Value)
disp("Total Value (TV):")
tv_dsr = sum(abs(dsr_data.u_array),2)'
tv_dsr_e = sum(abs(dsr_e_data.u_array),2)'
tv_pem = sum(abs(pem_data.u_array),2)'

% Plot setpoint tracking
figure;

% x - DSR
subplot(3,3,1);
hold on
plot(dsr_data.t_array, dsr_data.x_array(1,:));
plot(dsr_data.t_array, dsr_data.setpoint(1,:));
grid();
title('x (North) - DSR)');
xlabel('t [s]');
ylabel('North, x [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% x - DSR_e
subplot(3,3,2);
hold on
plot(dsr_e_data.t_array, dsr_e_data.x_array(1,:));
plot(dsr_e_data.t_array, dsr_e_data.setpoint(1,:));
grid();
title('x (North) - DSR_e)');
xlabel('t [s]');
ylabel('North, x [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% x - PEM
subplot(3,3,3);
hold on
plot(pem_data.t_array, pem_data.x_array(1,:));
plot(pem_data.t_array, pem_data.setpoint(1,:));
grid();
title('x (North) - PEM)');
xlabel('t [s]');
ylabel('North, x [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% y - DSR
subplot(3,3,4);
hold on
plot(dsr_data.t_array, dsr_data.x_array(2,:));
plot(dsr_data.t_array, dsr_data.setpoint(2,:));
grid();
title('y (East) - DSR)');
xlabel('t [s]');
ylabel('East, y [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% y - DSR_e
subplot(3,3,5);
hold on
plot(dsr_e_data.t_array, dsr_e_data.x_array(2,:));
plot(dsr_e_data.t_array, dsr_e_data.setpoint(2,:));
grid();
title('x (East) - DSR_e)');
xlabel('t [s]');
ylabel('East, y [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% y - PEM
subplot(3,3,6);
hold on
plot(pem_data.t_array, pem_data.x_array(2,:));
plot(pem_data.t_array, pem_data.setpoint(2,:));
grid();
title('x (East) - PEM)');
xlabel('t [s]');
ylabel('East, y [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% psi - DSR
subplot(3,3,7);
hold on
plot(dsr_data.t_array, dsr_data.x_array(3,:));
plot(dsr_data.t_array, dsr_data.setpoint(3,:));
grid();
title('psi (Heading) - DSR)');
xlabel('t [s]');
ylabel('Heading, psi [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% y - DSR_e
subplot(3,3,8);
hold on
plot(dsr_e_data.t_array, dsr_e_data.x_array(3,:));
plot(dsr_e_data.t_array, dsr_e_data.setpoint(3,:));
grid();
title('psi (Heading) - DSR_e)');
xlabel('t [s]');
ylabel('Heading, psi [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off

% y - PEM
subplot(3,3,9);
hold on
plot(pem_data.t_array, pem_data.x_array(3,:));
plot(pem_data.t_array, pem_data.setpoint(3,:));
grid();
title('psi (Heading) - PEM)');
xlabel('t [s]');
ylabel('Heading, psi [m]');
legend({'OSV', 'Setpoint'}, 'Location', 'Best');
hold off
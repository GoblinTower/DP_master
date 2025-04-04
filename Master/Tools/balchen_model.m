function x_dot = balchen_model(x, u, vc, Nc, Fw)
% Implementation of modified balchen model, including NED coordinates in
% the x variable. See the seminal paper by Balchen et al: "A dynamic
% positioning system based on Kalman filtering and optimal control".
%
% INPUT
% x(1)               : North position (x)
% x(2)               : East position (y)
% x(3)               : Vessel heading relative to north (psi)
% x(4)               : Distance travelled in surge direction (x_su)
% x(5)               : Distance travelled in sway direction (y_su)
% x(6)               : Velocity in surge (u)
% x(7)               : Velocity in sway (v)
% x(8)               : Vessel heading angular velocity (r)
% u(1)               : Thruster force in surge 
% u(2)               : Thruster force in sway
% u(3)               : Thruster momentum in yaw
% vc(1)              : Current velocity in surge
% vc(2)              : Current velocity in sway
% Nc                 : Current momentum in yaw
% Fw(1)              : Wind force in surge
% Fw(2)              : Wind force in sway
% Fw(3)              : Wind momentum in yaw
%
% OUTPUT:
% x_dot              : Time derivative of x vector
%

% Parameters
d1 = 5e-5;
d2 = 22e-5;
d3 = 12e-11; % Believe this should be negative
d4 = 225e-15;
m1 = 2.4e7;
m2 = 4e7;
m3 = 4.5e10;

x_dot = zeros(8,1);

% Acceleration
x_dot(6) = -d1/m1*abs(x(6) - vc(1))*(x(6) - vc(1)) + 1/m1*(Fw(1) + u(1));
x_dot(7) = -d2/m2*abs(x(7) - vc(2))*(x(7) - vc(2)) + 1/m2*(Fw(2) + u(2));
x_dot(8) = -d3/m3*abs(x(8))*x(8) - d4/m3*abs(x(7) - vc(2))*(x(7) - vc(2)) + 1/m3*(Nc + Fw(3) + u(3));

% Velocity
x_dot(4) = x(6);
x_dot(5) = x(7);

% Calculate rotation matrix
psi = x(3);
rotation = rotation_matrix(psi);

NED = rotation*[x(6); x(7); x(8)];
x_dot(1) = NED(1);
x_dot(2) = NED(2);
x_dot(3) = NED(3);

end
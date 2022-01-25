addpath(fullfile('src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% MPC_Control_x

% Design MPC controller
H = 2; % Horizon length in seconds
mpc_x = MPC_Control_x(sys_x, Ts, H);

% Get control input
x = [0, 0, 0, 0]';
x_position_reference = -5;
ux = mpc_x.get_u(x, x_position_reference)

x0 = [0, 0, 0, 0]';
x_ref = -5;
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);

%% MPC_Control_y

% Design MPC controller
H = 2; % Horizon length in seconds
mpc_y = MPC_Control_y(sys_y, Ts, H);

% Get control input
y = [0, 0, 0, 0]';
y_position_reference = -5;
uy = mpc_y.get_u(y, y_position_reference)

y0 = [0, 0, 0, 0]';
y_ref = -5;
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_y, y0, Tf, @mpc_y.get_u, y_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, y_ref);

%% MPC_Control_z

% Design MPC controller
H = 2; % Horizon length in seconds
mpc_z = MPC_Control_z(sys_z, Ts, H);

% Get control input
z = [0, 0]';
z_position_reference = -5;
uz = mpc_z.get_u(z, z_position_reference)

z0 = [0, 0]';
z_ref = -5;
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_z, z0, Tf, @mpc_z.get_u, z_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, z_ref);

%% MPC_Control_roll

% Design MPC controller
H = 2; % Horizon length in seconds
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Get control input
roll = [0, 0]';
roll_position_reference = pi/4;
uroll = mpc_roll.get_u(roll, roll_position_reference)

roll0 = [0, 0]';
roll_ref = pi/4;
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_roll, roll0, Tf, @mpc_roll.get_u, roll_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, roll_ref);


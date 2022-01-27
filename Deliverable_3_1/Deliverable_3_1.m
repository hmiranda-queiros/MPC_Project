clc; clear; close all;
addpath(fullfile('..','src'));

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20; % Sample time
rocket = Rocket(Ts);

[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 5; % Horizon length in seconds

%% MPC_Control_x

% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts, H);

x0 = [0, 0, 0, 5]';
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% MPC_Control_y

% Design MPC controller
mpc_y = MPC_Control_y(sys_y, Ts, H);

y0 = [0, 0, 0, 5]';
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_y, y0, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%% MPC_Control_z

% Design MPC controller
mpc_z = MPC_Control_z(sys_z, Ts, H);

z0 = [0, 5]';
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_z, z0, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);

%% MPC_Control_roll

% Design MPC controller
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

roll0 = [0, pi/4]';
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);


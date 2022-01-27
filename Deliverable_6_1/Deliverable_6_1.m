clc; clear; close all;
addpath(fullfile('..','src'));

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);

%% Non Linear Case
H = 4;
nmpc = NMPC_Control(rocket, H);

% MPC reference with default maximum roll = 15 deg
Tf = 30;
% ref = @(t_, x_) rocket.MPC_ref(t_, Tf);

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) rocket.MPC_ref(t_, Tf, roll_max);

x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

% Plot pose
rocket.anim_rate = 8; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title

%% Linear Case with roll_max = 50
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller
Ts = 1/20;
H = 2; % Horizon length in seconds

mpc_x = MPC_Control_x(sys_x, Ts, H);
mpc_y = MPC_Control_y(sys_y, Ts, H);
mpc_z = MPC_Control_z(sys_z, Ts, H);
mpc_roll = MPC_Control_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Setup reference function
Tf = 30;
roll_max = deg2rad(50);
ref = @(t_, x_) rocket.MPC_ref(t_, Tf, roll_max);
x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);

% Plot pose
rocket.anim_rate = 8; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
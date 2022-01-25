%% 1.1
Ts = 1/20;
rocket = Rocket(Ts);
d1 = 0;
d2 = 0;
Pavg = 50;
Pdiff = 0;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u)

w = [0, 0, 10];
phi = [0, 0, 0];
v = [0, 0, 10];
p = [0, 0, 10];
x = [w, phi, v, p]'; % (Assign appropriately)
x_dot = rocket.f(x, u)



%% 1.2

rocket = Rocket(Ts);
Tf = 2.0; % Time to simulate for
x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
% u = [deg2rad([0 0]), 60, 0 ]'; % ascend
% u = [deg2rad([0 0]), 0, 0 ]'; % descend
%u = [deg2rad([0 0]), 60, 20 ]'; % rotate z
%u = [deg2rad([0 0]), 56.5, 0 ]'; % hover
%u = [deg2rad([15 0]), 60, 0 ]'; % rotate y
%u = [deg2rad([0 15]), 60, 0 ]'; % rotate x
%u = [deg2rad([1 0]), 80, 0 ]'; % fly y
u = [deg2rad([0 1]), 80, 0 ]'; % fly x


[T, X, U] = rocket.simulate_f(x0, Tf, u); % Simulate nonlinear dynamics f
rocket.anim_rate = 1.0;
rocket.vis(T, X, U); % Trajectory visualization at 1.0x real−time


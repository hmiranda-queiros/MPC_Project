function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%f_discrete = @(X,U) ode45(@(t,x) rocket.f(x,U),[0 rocket.Ts], X');
f_discrete = @(x,u) RK4(x, u, rocket.Ts, rocket.f);

% Dynamic constraints and objective
obj = 0;

opti.subject_to(X_sym(:,2) == f_discrete(X_sym(:,1), U_sym(:,1)));
opti.subject_to(M*U_sym(:,1) <= m);
obj = obj + U_sym(:,1)'*R*U_sym(:,1); 

for i = 2:N-1
    opti.subject_to(X_sym(:, i+1) == F(X_sym(:,i), U_sym(:,i)));
    opti.subject_to(F*X_sym(:,i) <= f);
    opti.subject_to(M*U_sym(:,i) <= m);
    obj = obj + (X_sym(:,i) - x_ref)'*Q*(X_sym(:,i) - x_ref) + (U_sym(:,i) - u_ref)'*R*(U_sym(:,i) - u_ref);
end

% Initial conditions
opti.subject_to(x(:,1) == x0_sym);

% Terminal Cost
obj = obj + (X_sym(:,N) - x_ref)'*Qf*(X_sym(:,N) - x_ref);

% Objective
opti.minimize(obj);



% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end

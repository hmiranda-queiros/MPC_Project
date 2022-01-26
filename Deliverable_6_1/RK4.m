function [x_next] = RK4(X,U,rocket)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%
h = rocket.Ts;


% Runge-Kutta 4 integration

k1 = rocket.f(X, U);
k2 = rocket.f(X+h/2*k1, U);
k3 = rocket.f(X+h/2*k2, U);
k4 = rocket.f(X+h*k3, U);
x_next = X + h/6*(k1+2*k2+2*k3+k4);

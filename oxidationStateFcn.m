function dxdt = oxidationStateFcn(x,u)

% System consists of 4 states (x): 
% x1 = Gas density in reactor
% x2 = Ethylene (reactant) concentration in reactor
% x3 = Ethylene oxide (product) concentration in reactor
% x4 = Temperature in reactor

% Input (u) consists of 2 parts
% u1 = Volumetric feed flow rate
% u2 = Ethylene concentration in the feed flow

% 1 output (y)  
% y = Ethylene oxide concentration in the effluent flow (equivalent to x3)



%% Parameters
gamma_1 = -8.13;
gamma_2 = -7.12;
gamma_3 = -11.07;
A_1 = 92.80;
A_2 = 12.66;
A_3 = 2412.71;
B_1 = 7.32;
B_2 = 10.39;
B_3 = 2170.57;
B_4 = 7.02;
Tc = 1.0;

%% Initialization
dxdt = zeros(4,1);    % dxdt is the derivative of the states. within 4x1 matrix

%% Kinetic rate expressions
rate1 = exp(gamma_1/x(4))*(x(2)*x(4))^0.5;
rate2 = exp(gamma_2/x(4))*(x(2)*x(4))^0.25;
rate3 = exp(gamma_3/x(4))*(x(3)*x(4))^0.5;

%% ODEs based on mass and energy balances
dxdt(1) = u(1)*(1 - x(1)*x(4));
dxdt(2) = u(1)*(u(2) - x(2)*x(4)) - A_1*rate1 - A_2*rate2;
dxdt(3) = -u(1)*x(3)*x(4) + A_1*rate1 - A_3*rate3;
dxdt(4) = (u(1)*(1 - x(4)) + B_1*rate1 + B_2*rate2 + B_3*rate3 - B_4*(x(4) - Tc))/x(1);

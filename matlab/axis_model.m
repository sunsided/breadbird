function [Ad, Bd, Ac, Bc] = axis_model(b, tau, T)
% AXIS_MODEL  One attitude axis, discretized with a zero-order hold.
%
%   [Ad, Bd, Ac, Bc] = axis_model(b, tau, T)
%
% Continuous model (states theta [rad], omega [rad/s], Td [N]):
%
%   theta' = omega
%   omega' = b Td              b = l / J, angular acceleration per newton
%   Td'    = (u - Td) / tau    motor lag: the differential thrust Td
%                              follows the commanded u with time constant tau
%
% The discrete model holds u constant over one sample period T and adds a
% fourth state, the input of the previous sample, for a one-sample delay
% between computing u and the motors seeing it:
%
%   x = [theta; omega; Td; u_prev],   x[k+1] = Ad x[k] + Bd u[k]
%
% Ad and Bd are written out in closed form (the same formulas the Rust crate
% uses); attitude_control.m checks them against expm.

a = exp(-T / tau);  % motor pole in the z-plane
g = 1 - a;          % fraction of a step the motor follows within one sample

Ad3 = [1, T, b*tau*(T - tau*g);
       0, 1, b*tau*g;
       0, 0, a];
Bd3 = [b*(T^2/2 - tau*(T - tau*g));
       b*(T - tau*g);
       g];

Ad = [Ad3, Bd3; zeros(1, 4)];
Bd = [0; 0; 0; 1];

Ac = [0, 1, 0; 0, 0, b; 0, 0, -1/tau];
Bc = [0; 0; 1/tau];
end

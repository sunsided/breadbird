function [P, iterations] = dare_sda(A, B, Q, R)
% DARE_SDA  Solve the discrete algebraic Riccati equation
%
%   P = A' P A - A' P B (R + B' P B)^-1 B' P A + Q
%
% with the structure-preserving doubling algorithm. Plain Riccati iteration
% (P <- right-hand side) needs one step per sample of the cost horizon, which
% at 500 Hz means thousands of steps; doubling squares the horizon in each
% step and converges in a few dozen.
%
% The LQR gain follows as K = (R + B' P B)^-1 B' P A. For the steady-state
% Kalman filter, call it with (A', H', Q, R): P is then the prior covariance.

n = size(A, 1);
Ak = A;
Gk = B * (R \ B');
Hk = Q;
for iterations = 1:200
    W = eye(n) + Gk * Hk;
    Ak1 = Ak * (W \ Ak);
    Gk1 = Gk + Ak * (W \ Gk) * Ak';
    Hk1 = Hk + Ak' * Hk * (W \ Ak);
    done = norm(Hk1 - Hk, 1) <= 1e-13 * norm(Hk1, 1);
    Ak = Ak1;
    Gk = (Gk1 + Gk1') / 2;
    Hk = (Hk1 + Hk1') / 2;
    if done
        break;
    end
end
P = Hk;
end

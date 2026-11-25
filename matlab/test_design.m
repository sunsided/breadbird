% Differential tests: the hand-rolled helpers used by attitude_control.m
% against the Octave control package (pkg install -forge control).
%
%   octave --no-gui matlab/test_design.m

clear all;
addpath(fileparts(mfilename('fullpath')));
pkg load control;

relerr = @(a, b) norm(a - b) / max(norm(b), eps);
checks = 0;
check = @(name, e, tol) assert(e < tol, '%s: relative error %.3g >= %.3g', name, e, tol);

for T = [0.002, 0.005, 0.02]
    for tau = [0.015, 0.03, 0.08]
        b = 34;
        [Ad, Bd, Ac, Bc] = axis_model(b, tau, T);

        % zero-order hold discretization, delay state appended
        sysd = c2d(ss(Ac, Bc, eye(3), 0), T, 'zoh');
        check('axis_model A', relerr(Ad(1:3, 1:3), sysd.a), 1e-12);
        check('axis_model B', relerr(Ad(1:3, 4), sysd.b), 1e-12);
        checks = checks + 2;

        % pole placement against place()
        wn = 15; zeta = 0.7;
        p = [exp(T * wn * (-zeta + 1i * sqrt(1 - zeta^2))), ...
             exp(T * wn * (-zeta - 1i * sqrt(1 - zeta^2))), exp(-T * 40), 0];
        check('place_ackermann', relerr(place_ackermann(Ad, Bd, p), place(Ad, Bd, p)), 1e-6);
        checks = checks + 1;

        % LQR against dlqr(), with and without integral state
        Q = diag([400, 1, 0, 0]); R = 4;
        [K_ref, P_ref] = dlqr(Ad, Bd, Q, R);
        P = dare_sda(Ad, Bd, Q, R);
        K = (R + Bd' * P * Bd) \ (Bd' * P * Ad);
        check('dare_sda P', relerr(P, P_ref), 1e-8);
        check('dare_sda K', relerr(K, K_ref), 1e-8);
        A5 = [Ad, zeros(4, 1); -T, 0, 0, 0, 1]; B5 = [Bd; 0]; Q5 = blkdiag(Q, 1e4);
        P5 = dare_sda(A5, B5, Q5, R);
        check('dare_sda K5', relerr((R + B5' * P5 * B5) \ (B5' * P5 * A5), dlqr(A5, B5, Q5, R)), 1e-8);
        checks = checks + 3;

        % steady-state Kalman gain by duality against dlqe()
        H = [1 0 0 0; 0 1 0 0];
        Qn = diag([0, 1e-4, 1e-2, 0]); Rn = diag([0.002, 0.01] .^ 2);
        Pn = dare_sda(Ad', H', Qn, Rn);
        Kf = Pn * H' / (H * Pn * H' + Rn);
        [Kf_ref, Pn_ref] = dlqe(Ad, eye(4), H, Qn, Rn);
        check('observer prior P', relerr(Pn, Pn_ref), 1e-8);
        check('observer Kf', relerr(Kf, Kf_ref), 1e-8);
        checks = checks + 2;
    end
end

% twiddle finds the minimum of a bowl
[p, c] = twiddle(@(p) sum((p - [1 -2 3]) .^ 2), [0 0 0], [1 1 1], 1e-6);
check('twiddle', norm(p - [1 -2 3]), 1e-3);
checks = checks + 1;

fprintf('test_design: %d checks passed\n', checks);

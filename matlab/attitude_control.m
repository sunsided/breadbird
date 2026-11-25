% Attitude control design desk: one axis of the quadrotor (chapter "State-space
% control" in typst/chapters/control.typ).
%
% Designs pole placement, LQR, integral action, an observer and PID variants
% for the reference airframe below, simulates them and writes the results as
% CSV to ../typst/data/control/ for the figures. Also prints the gains that
% the Rust crate in ../control/ checks itself against. No control package
% needed; the helpers next to this file do the work:
%
%   octave --no-gui matlab/attitude_control.m
%
% REFERENCE AIRFRAME: placeholder values for a ~0.5 kg quadrotor. Replace them
% with the values identified for the real vehicle (chapter "Identifying the
% plant").

clear all;
addpath(fileparts(mfilename('fullpath')));
randn('state', 42);

l   = 0.17;    % arm length [m]
J   = 5e-3;    % moment of inertia about the roll axis [kg m^2]
tau = 0.03;    % motor time constant [s]
T   = 0.002;   % sample period [s] (500 Hz)
b   = l / J;   % angular acceleration per newton of differential thrust [rad/s^2/N]
umax = 1.0;    % largest differential thrust command [N]

[Ad, Bd, Ac, Bc] = axis_model(b, tau, T);
H = [1 0 0 0; 0 1 0 0];   % measured: angle (attitude estimate) and rate (gyro)
noise = [0.002; 0.01];    % measurement noise [rad; rad/s]

% The closed-form discretization must match the matrix exponential.
M = expm([Ac, Bc; zeros(1, 4)] * T);
assert(norm(Ad(1:3, 1:3) - M(1:3, 1:3)) < 1e-12 && norm(Ad(1:3, 4) - M(1:3, 4)) < 1e-12);

fprintf('b = %.6g rad/s^2/N, motor pole a = %.9g\n', b, Ad(3, 3));
fprintf('open-loop eigenvalues: %s\n', mat2str(eig(Ad)', 6));

%% Open loop: a short thrust pulse tips the vehicle over for good
N = 500;
u_pulse = zeros(N, 1);
u_pulse(1:25) = 0.05;
x = zeros(4, 1);
ol = zeros(N, 3);
for k = 1:N
    ol(k, :) = [(k - 1) * T, x(1), x(2)];
    x = Ad * x + Bd * u_pulse(k);
end
write_csv('open_loop', 't,theta,omega', ol);

%% Observer: steady-state Kalman gain via the dual Riccati equation
% Process noise mostly on the motor (thrust ripple, turbulence), a little on
% the rate; the delayed input is known exactly.
Qn = diag([0, 1e-4, 1e-2, 0]);
Rn = diag(noise .^ 2);
Pn = dare_sda(Ad', H', Qn, Rn);
Kf = Pn * H' / (H * Pn * H' + Rn);   % current form: xhat = xbar + Kf (y - H xbar)
L = Ad * Kf;                         % predictor form gain, as minikalman expects it
fprintf('observer eigenvalues |A - L H|: %s\n', mat2str(abs(eig(Ad - L * H))', 4));

sf = struct('kind', 'sf', 'T', T, 'A', Ad, 'B', Bd, 'H', H, 'Kf', Kf, ...
            'xhat', zeros(4, 1), 'u', 0, 'xi', 0, 'ki', 0, 'umax', umax, ...
            'antiwindup', true, 'r', 0.1);

%% Pole placement
% Dominant pair from natural frequency wn and damping zeta, the motor pole a
% bit faster than the motor itself, the delay state at z = 0.
pp = @(wn, zeta) [exp(T * wn * (-zeta + 1i * sqrt(1 - zeta^2))), ...
                  exp(T * wn * (-zeta - 1i * sqrt(1 - zeta^2))), ...
                  exp(-T * 40), 0];

[K_pp, cond_C] = place_ackermann(Ad, Bd, pp(15, 0.7));
assert(norm(sort(abs(eig(Ad - Bd * K_pp))) - sort(abs(pp(15, 0.7)'))) < 1e-6);
fprintf('cond(controllability matrix) at %g Hz: %.3g\n', 1 / T, cond_C);
[~, cond_C50] = place_ackermann(axis_model(b, tau, 0.02), [0; 0; 0; 1], pp(15, 0.7));
fprintf('cond(controllability matrix) at 50 Hz: %.3g\n', cond_C50);
fprintf('K_pp (wn = 15, zeta = 0.7) = %s\n', mat2str(K_pp, 12));

N = 750;
cols = [];
hyp = [];
% Hypothetical model error: motors twice as slow as modeled and 8 ms (four
% samples) more delay, e.g. from the motor controllers; same gains, full state.
As_slow = axis_model(b, 2 * tau, T);
extra = 4;
for wn = [8 15 25 40]
    s = sf; s.K = place_ackermann(Ad, Bd, pp(wn, 0.7));
    o = sim_axis(Ad, Bd, N, s, 0, [0; 0]);
    cols = [cols, o(:, 2), o(:, 5)];
    x = zeros(3, 1); queue = zeros(1, extra + 1); u_prev = 0; th = zeros(N, 1);
    for k = 1:N
        th(k) = x(1);
        u = min(max(-s.K * ([x; u_prev] - [s.r; 0; 0; 0]), -umax), umax);
        queue = [u, queue(1:end - 1)];
        x = As_slow(1:3, 1:3) * x + As_slow(1:3, 4) * queue(end);
        u_prev = u;
    end
    hyp = [hyp, th];
end
write_csv('pp_wn', 't,theta_8,u_8,theta_15,u_15,theta_25,u_25,theta_40,u_40,theta_8_err,theta_15_err,theta_25_err,theta_40_err', [o(:, 1), cols, hyp]);

cols = [];
for zeta = [0.3 0.7 1.0]
    s = sf; s.K = place_ackermann(Ad, Bd, pp(15, zeta));
    o = sim_axis(Ad, Bd, N, s, 0, [0; 0]);
    cols = [cols, o(:, 2)];
end
write_csv('pp_zeta', 't,theta_03,theta_07,theta_10', [o(:, 1), cols]);

% Moving the motor pole: same dominant pair, motor pole at -40, -130 and -400 1/s,
% with saturation and measurement noise as on the real vehicle
cols = [];
r_motor = [zeros(25, 1); 0.1 * ones(N - 25, 1)];
for pm = [40 130 400]
    s = sf; s.K = place_ackermann(Ad, Bd, [pp(15, 0.7)(1:2), exp(-T * pm), 0]);
    s.r = r_motor;
    randn('state', 3);
    o = sim_axis(Ad, Bd, N, s, 0, noise);
    cols = [cols, o(:, 2), o(:, 5)];
    s.umax = 1e9; o_lin = sim_axis(Ad, Bd, N, s, 0, [0; 0]);
    fprintf('motor pole -%d: K = %s, peak command unlimited %.2f N, command noise %.3f N, saturated %.0f %%\n', ...
            pm, mat2str(s.K, 3), max(abs(o_lin(:, 5))), std(cols(500:end, end)), 100 * mean(abs(cols(:, end)) >= umax - 1e-9));
end
write_csv('pp_motor', 't,theta_40,u_40,theta_130,u_130,theta_400,u_400', [o(:, 1), cols]);

% Where these poles end up in the z-plane, at 500 Hz and at 50 Hz
zp = [];
for wn = [8 15 25]
    for zeta = [0.3 0.7 1.0]
        for Ts = [T 0.02]
            z = exp(Ts * wn * (-zeta + 1i * sqrt(1 - zeta^2)));
            zp(end + 1, :) = [wn, zeta, 1 / Ts, real(z), imag(z)];
        end
    end
end
write_csv('z_poles', 'wn,zeta,fs,re,im', zp);

%% LQR with Bryson's rule: weight each state by 1 / (largest acceptable value)^2
theta_max = 0.05;   % [rad]
omega_max = 1.0;    % [rad/s]
Q = diag([1 / theta_max^2, 1 / omega_max^2, 0, 0]);
R = 1 / 0.5^2;      % accept 0.5 N of differential thrust
lqr = @(Q, R) (R + Bd' * dare_sda(Ad, Bd, Q, R) * Bd) \ (Bd' * dare_sda(Ad, Bd, Q, R) * Ad);

[P, iters] = dare_sda(Ad, Bd, Q, R);
res = Ad' * P * Ad - Ad' * P * Bd / (R + Bd' * P * Bd) * Bd' * P * Ad + Q - P;
assert(norm(res) < 1e-8 * norm(P));
K_lqr = lqr(Q, R);
fprintf('DARE: %d doubling steps, residual %.3g\n', iters, norm(res));
fprintf('K_lqr = %s, closed-loop |z| = %s\n', mat2str(K_lqr, 12), mat2str(abs(eig(Ad - Bd * K_lqr))', 4));

cols = [];
for rho = [0.1 1 10]
    s = sf; s.K = lqr(Q, rho * R);
    o = sim_axis(Ad, Bd, N, s, 0, [0; 0]);
    cols = [cols, o(:, 2), o(:, 5)];
end
write_csv('lqr_rho', 't,theta_01,u_01,theta_1,u_1,theta_10,u_10', [o(:, 1), cols]);

%% Closed-loop poles in the s-plane, s = ln(z) / T (the delay pole z = 0 has no s)
to_s = @(z) log(z(abs(z) > 1e-9)) / T;
cl = [];
for wn = [8 15 25]
    s_ = to_s(eig(Ad - Bd * place_ackermann(Ad, Bd, pp(wn, 0.7))));
    cl = [cl; repmat(wn, numel(s_), 1), zeros(numel(s_), 1), real(s_), imag(s_)];
end
s_ = to_s(eig(Ad - Bd * K_lqr));
cl = [cl; zeros(numel(s_), 1), ones(numel(s_), 1), real(s_), imag(s_)];
s_ = to_s(eig(Ad - L * H));
cl = [cl; zeros(numel(s_), 1), 2 * ones(numel(s_), 1), real(s_), imag(s_)];
write_csv('cl_poles', 'wn,kind,re,im', cl);

% LQR pole locus: how the closed-loop poles move as the input weight R is scaled
locus = [];
for rho = logspace(-3, 3, 61)
    s_ = to_s(eig(Ad - Bd * lqr(Q, rho * R)));
    for j = 1:numel(s_)
        locus(end + 1, :) = [rho, real(s_(j)), imag(s_(j))];
    end
end
write_csv('lqr_locus', 'rho,re,im', locus);

%% Integral action: augment with xi[k+1] = xi[k] + T (r - theta[k])
A5 = [Ad, zeros(4, 1); -T, 0, 0, 0, 1];
B5 = [Bd; 0];
Q5 = blkdiag(Q, 1 / 0.01^2);
P5 = dare_sda(A5, B5, Q5, R);
K5 = (R + B5' * P5 * B5) \ (B5' * P5 * A5);
fprintf('K_lqr_i = %s\n', mat2str(K5, 12));

N = 1500;
r = [zeros(50, 1); 0.1 * ones(N - 50, 1)];
dist = [zeros(750, 1); 0.05 * ones(N - 750, 1)];   % 5 g of thrust offset after 1.5 s
s = sf; s.K = K_lqr; s.r = r;
o_p = sim_axis(Ad, Bd, N, s, dist, noise);
s = sf; s.K = K5(1:4); s.ki = -K5(5); s.r = r;
o_i = sim_axis(Ad, Bd, N, s, dist, noise);
write_csv('integral', 't,r,theta_p,theta_i,u_p,u_i,td,td_hat', ...
          [o_p(:, 1), r, o_p(:, 2), o_i(:, 2), o_p(:, 5), o_i(:, 5), o_i(:, 4), o_i(:, 6)]);

%% PID on the angle, with the gyro as derivative: u = Kp e + Ki int(e) - Kd omega
% That is state feedback K = [Kp, Kd, 0, 0] plus integral action. Taking Kp and
% Kd from the LQR gain gives a PID that ignores motor lag and delay.
pid = struct('kind', 'pid', 'T', T, 'var', 1, 'gyro', true, ...
             'Kp', K5(1), 'Kd', K5(2), 'Ki', -K5(5), 'N', 10, ...
             'I', 0, 'D', 0, 'u', 0, 'e_prev', 0, 'umax', umax, 'antiwindup', true, 'r', r);
fprintf('angle PID from LQR: Kp = %.4g, Ki = %.4g, Kd = %.4g\n', pid.Kp, pid.Ki, pid.Kd);
o_pid = sim_axis(Ad, Bd, N, pid, dist, noise);

% Incremental form gives the same output as long as nothing saturates
randn('state', 1); a1 = sim_axis(Ad, Bd, 500, setfield(pid, 'umax', 1e9), 0, noise);
pinc = pid; pinc.kind = 'pid_inc'; pinc.umax = 1e9;
randn('state', 1); a2 = sim_axis(Ad, Bd, 500, pinc, 0, noise);
assert(max(abs(a1(:, 5) - a2(:, 5))) < 1e-9);

% Windup: a large step against a tight output limit
N = 1500;
big = [zeros(50, 1); 0.8 * ones(N - 50, 1)];
w = pid; w.umax = 0.15; w.r = big;
w.antiwindup = false; o_w0 = sim_axis(Ad, Bd, N, w, 0, [0; 0]);
w.antiwindup = true;  o_w1 = sim_axis(Ad, Bd, N, w, 0, [0; 0]);
w.kind = 'pid_inc';   o_w2 = sim_axis(Ad, Bd, N, w, 0, [0; 0]);
write_csv('windup', 't,r,theta_none,theta_clamp,theta_inc,u_none,u_clamp', ...
          [o_w0(:, 1), big, o_w0(:, 2), o_w1(:, 2), o_w2(:, 2), o_w0(:, 5), o_w1(:, 5)]);

%% Rate loop tuning: ultimate gain (Ziegler-Nichols) and SIMC
% P control of the rate: u = Kp (r - omega). Increase Kp until the loop
% oscillates without decaying (largest |eigenvalue| = 1). The angle is not
% part of the rate loop (its integrator stays at z = 1), so only the
% omega/Td/u_prev block counts.
Krate = @(kp) [0, kp, 0, 0];
rate_eig = @(kp) eig(Ad(2:4, 2:4) - Bd(2:4) * Krate(kp)(2:4));
lo = 0; hi = 10;
for i = 1:60
    mid = (lo + hi) / 2;
    if max(abs(rate_eig(mid))) < 1
        lo = mid;
    else
        hi = mid;
    end
end
Ku = lo;
ev = rate_eig(Ku);
[~, i] = max(abs(ev) .* (imag(ev) > 0));
Tu = 2 * pi * T / angle(ev(i));
fprintf('rate loop: Ku = %.4g N/(rad/s), Tu = %.4g s\n', Ku, Tu);
zn = [0.6 * Ku, 1.2 * Ku / Tu, 0.075 * Ku * Tu];   % [Kp Ki Kd], classic ZN PID

% SIMC for an integrating process with lag, k e^(-theta s) / (s (tau s + 1)):
% series PID Kc = 1 / (k (tc + theta)), tI = 4 (tc + theta), tD = tau.
theta_d = 1.5 * T;   % one sample delay plus half a sample from the zero-order hold
tc = 0.02;           % desired closed-loop time constant [s]
Kc = 1 / (b * (tc + theta_d)); tI = 4 * (tc + theta_d); tD = tau;
simc = [Kc * (1 + tD / tI), Kc * (1 + tD / tI) / (tI + tD), Kc * (1 + tD / tI) * tI * tD / (tI + tD)];
fprintf('rate PID ZN:   Kp = %.4g, Ki = %.4g, Kd = %.4g\n', zn);
fprintf('rate PID SIMC: Kp = %.4g, Ki = %.4g, Kd = %.4g\n', simc);

rate = struct('kind', 'pid', 'T', T, 'var', 2, 'N', 10, 'I', 0, 'D', 0, 'u', 0, ...
              'e_prev', 0, 'umax', umax, 'antiwindup', true);
N = 500;
r_rate = [zeros(10, 1); ones(N - 10, 1)];   % 1 rad/s rate step
d_rate = [zeros(250, 1); 0.05 * ones(N - 250, 1)];   % thrust offset after 0.5 s
rate_run = @(p) sim_rate(Ad, Bd, N, rate, p, r_rate, d_rate, noise);

% Twiddle against the simulated plant: time-weighted absolute error plus a
% penalty on how much the output jumps (the gyro noise it passes on to the
% motors), starting from SIMC. Same noise sequence for every evaluation.
cost = @(p) rate_cost(rate_run(p), r_rate, T, p);
[p_tw, c_tw, hist] = twiddle(cost, simc, simc / 4, 1e-4 * sum(simc));
fprintf('rate PID twiddle: Kp = %.4g, Ki = %.4g, Kd = %.4g (cost %.4g -> %.4g, %d sweeps)\n', ...
        p_tw, cost(simc), c_tw, size(hist, 1) - 1);
write_csv('twiddle', 'sweep,cost,kp,ki,kd', hist);

o_zn = rate_run(zn); o_simc = rate_run(simc); o_tw = rate_run(p_tw);
write_csv('rate_tuning', 't,r,omega_zn,omega_simc,omega_tw,u_zn,u_simc,u_tw', ...
          [o_zn(:, 1), r_rate, o_zn(:, 3), o_simc(:, 3), o_tw(:, 3), o_zn(:, 5), o_simc(:, 5), o_tw(:, 5)]);

%% Cascade: P on the angle feeding the tuned rate PID, against LQR + integral
N = 1500;
inner = rate; inner.Kp = p_tw(1); inner.Ki = p_tw(2); inner.Kd = p_tw(3); inner.r = zeros(N, 1);
casc = struct('kind', 'cascade', 'T', T, 'Ktheta', 1 / (4 * tc), ...
              'rate_max', 3, 'inner', inner, 'r', r);
fprintf('cascade: Ktheta = %.4g 1/s\n', casc.Ktheta);
o_c = sim_axis(Ad, Bd, N, casc, dist, noise);
write_csv('cascade', 't,r,theta_cascade,theta_pid,theta_lqr_i,u_cascade,u_pid,u_lqr_i', ...
          [o_c(:, 1), r, o_c(:, 2), o_pid(:, 2), o_i(:, 2), o_c(:, 5), o_pid(:, 5), o_i(:, 5)]);

%% Reference values for the Rust crate (control/tests/octave_reference.rs)
fprintf('\n// reference: b = %.17g, tau = %.17g, T = %.17g\n', b, tau, T);
fprintf('Ad = %s\n', mat2str(Ad, 17));
fprintf('Bd(1:3) as u_prev column = %s\n', mat2str(Ad(1:3, 4), 17));
fprintf('K_pp = %s\n', mat2str(K_pp, 17));
fprintf('K_lqr = %s\n', mat2str(K_lqr, 17));
fprintf('K_lqr_i = %s\n', mat2str(K5, 17));
fprintf('Kf = %s\n', mat2str(Kf, 17));

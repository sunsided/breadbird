function out = sim_axis(Ad, Bd, N, s, dist, noise)
% SIM_AXIS  Simulate one attitude axis under a controller from controller.m.
%
%   out = sim_axis(Ad, Bd, N, s, dist, noise)
%
% Runs N samples from rest. dist (scalar or length N) is a thrust offset added
% to the commanded input, e.g. a center of gravity off the axis; noise =
% [sigma_theta; sigma_omega] is the measurement noise. Columns of out:
% t, theta, omega, Td, u, Td_hat (NaN without observer), reference.

if numel(dist) == 1
    dist = dist * ones(N, 1);
end
x = zeros(4, 1);
out = zeros(N, 7);
for k = 1:N
    y = x(1:2) + noise(:) .* randn(2, 1);
    [u, s] = controller(s, y, k);
    td_hat = NaN;
    if isfield(s, 'xhat')
        td_hat = s.xhat(3);
    end
    out(k, :) = [(k - 1) * s.T, x(1), x(2), x(3), u, td_hat, s.r(min(k, numel(s.r)))];
    x = Ad * x + Bd * (u + dist(k));
end
end

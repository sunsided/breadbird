function c = rate_cost(out, r, T, p)
% RATE_COST  Cost of a rate step for twiddle: integral of time-weighted
% absolute error (ITAE) plus a penalty on sample-to-sample output changes,
% which is where amplified gyro noise shows up. Negative gains p cost Inf.
e = r - out(:, 3);
c = T * sum(out(:, 1) .* abs(e)) + 1e-2 * sum(diff(out(:, 5)) .^ 2);
if ~isfinite(c) || any(p < 0)
    c = Inf;
end
end

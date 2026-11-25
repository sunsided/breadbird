function [p, best, history] = twiddle(cost, p, dp, tol)
% TWIDDLE  Coordinate descent with adaptive step sizes (Thrun's "twiddle").
%
%   [p, best, history] = twiddle(cost, p, dp, tol)
%
% Tries p(i) + dp(i), then p(i) - dp(i), for each parameter in turn. A step
% that lowers the cost is kept and dp(i) grows by 10 %; if neither does,
% p(i) stays and dp(i) shrinks by 10 %. Stops when sum(dp) < tol. history
% holds one row [iteration, best cost, p] per sweep.

best = cost(p);
history = [0, best, p];
sweep = 0;
while sum(dp) > tol && sweep < 500
    sweep = sweep + 1;
    for i = 1:numel(p)
        p(i) = p(i) + dp(i);
        c = cost(p);
        if c < best
            best = c;
            dp(i) = dp(i) * 1.1;
            continue;
        end
        p(i) = p(i) - 2 * dp(i);
        c = cost(p);
        if c < best
            best = c;
            dp(i) = dp(i) * 1.1;
            continue;
        end
        p(i) = p(i) + dp(i);
        dp(i) = dp(i) * 0.9;
    end
    history(end + 1, :) = [sweep, best, p];
end
end

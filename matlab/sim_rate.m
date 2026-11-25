function out = sim_rate(Ad, Bd, N, rate, p, r, dist, noise)
% SIM_RATE  Rate PID with gains p = [Kp Ki Kd] on a rate step r with input
% disturbance dist; reseeds the noise so that every call sees the same
% sequence (twiddle compares costs between calls).
rate.Kp = p(1); rate.Ki = p(2); rate.Kd = p(3); rate.r = r;
randn('state', 7);
out = sim_axis(Ad, Bd, N, rate, dist, noise);
end

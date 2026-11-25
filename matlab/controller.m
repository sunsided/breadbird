function [u, s] = controller(s, y, k)
% CONTROLLER  One step of the controllers compared in attitude_control.m.
%
%   [u, s] = controller(s, y, k)
%
% y = [theta; omega] is the measurement at sample k, s the controller state
% (returned updated) and u the commanded differential thrust. s.kind selects:
%
%   'sf'       state feedback u = -K (xhat - xref) + ki xi, with a Luenberger
%              observer in current form (gain s.Kf) and optional integral xi
%   'pid'      positional PID on y(s.var), derivative on the measurement
%   'pid_inc'  the same PID in incremental (velocity) form
%   'cascade'  P on the angle commanding a rate setpoint for s.inner
%
% All of them saturate u to +-s.umax; s.antiwindup stops the integrator
% while the output is saturated in the direction the integrator pushes.

T = s.T;
r = s.r(min(k, numel(s.r)));

switch s.kind
    case 'sf'
        if k > 1
            xbar = s.A * s.xhat + s.B * s.u;
        else
            xbar = s.xhat;
        end
        s.xhat = xbar + s.Kf * (y - s.H * xbar);

        e = r - y(1);
        xi = s.xi + T * e;
        u_un = -s.K * (s.xhat - [r; 0; 0; 0]) + s.ki * xi;
        u = min(max(u_un, -s.umax), s.umax);
        if ~(s.antiwindup && u ~= u_un && sign(u_un) == sign(s.ki * e))
            s.xi = xi;
        end
        s.u = u;

    case {'pid', 'pid_inc'}
        meas = y(s.var);
        if k == 1
            s.meas_prev = meas;
        end
        e = r - meas;

        if isfield(s, 'gyro') && s.gyro
            % angle loop: the gyro measures the derivative of the angle
            D = -s.Kd * y(2);
        elseif s.Kd > 0
            % D on the measurement, low-pass filtered with Tf = Td / N
            Tf = s.Kd / s.Kp / s.N;
            D = Tf / (Tf + T) * s.D - s.Kd / (Tf + T) * (meas - s.meas_prev);
        else
            D = 0;
        end

        if strcmp(s.kind, 'pid')
            I = s.I + s.Ki * T * e;
            u_un = s.Kp * e + I + D;
            u = min(max(u_un, -s.umax), s.umax);
            if ~(s.antiwindup && u ~= u_un && sign(u_un) == sign(s.Ki * e))
                s.I = I;
            end
        else
            du = s.Kp * (e - s.e_prev) + s.Ki * T * e + (D - s.D);
            u = min(max(s.u + du, -s.umax), s.umax);
            s.e_prev = e;
            s.u = u;
        end
        s.D = D;
        s.meas_prev = meas;

    case 'cascade'
        rate_ref = min(max(s.Ktheta * (r - y(1)), -s.rate_max), s.rate_max);
        s.inner.r(k) = rate_ref;
        [u, s.inner] = controller(s.inner, y, k);
end
end

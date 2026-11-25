% Momentum theory in hover: outflow velocity and ideal power required to
% produce a fixed static thrust with rotors of different radius.
%
% Writes one CSV per thrust level to ../typst/data/mt_energy/<F>N.csv
% (columns r, A, Q, rho, mdot, v0, vi, v3, F, E) for the plots in
% typst/figures/mt_energy/. Runs in MATLAB and GNU Octave:
%
%   octave --no-gui matlab/momentum_theory_energy.m

clear all;

% fluid density
rho = 1.293; % [kg/m^3]

% target static thrust levels
forces = [0.25 0.5 1 1.5 2 4 8]; % [N]

% induced velocity (Froude-Rankine theorem)
vi = @(v3,v0) 0.5*(v3+v0);

% volumentric flow rate
Q = @(A,vi) A*vi;

% mass flow rate
mdot = @(A,vi) rho*Q(A,vi);

% theoretical dynamic thrust
F = @(A,v3,v0) mdot(A,vi(v3,v0))*(v3-v0);

% ideal (induced) power: kinetic energy added to the air per second [W]
E = @(A,v3,v0) F(A,v3,v0)*vi(v3,v0);

% outflow velocity from force and area (static case, v0 = 0)
v3N = @(F,A) sqrt(2*F./(rho*A));

% area from outflow velocity and force
AN = @(F,v3) 2*F./(rho*v3.^2);

% simulation
r_min = 0.01;       % radius [m]
r_max = 0.1;        % radius [m]
N_points = 100;     % nunber of sampling points

r_series = linspace(r_min,r_max,N_points); % radius [m]
A_series = pi*r_series.^2;

out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'typst', 'data', 'mt_energy');

figure;
hold on;
for force = forces
    v3_series = v3N(force, A_series);    % outflow velocity [m/s]
    v0 = 0;

    data = zeros(N_points, 10);
    for index = 1:N_points
        A = A_series(index);
        v3 = v3_series(index);
        vi_ = vi(v3,v0);
        data(index,:) = [r_series(index), A, Q(A,vi_), rho, mdot(A,vi_), v0, vi_, v3, force, E(A,v3,v0)];
    end

    % thrust must come out as requested: F = mdot * (v3 - v0)
    assert(max(abs(data(:,5) .* data(:,8) - force)) < 1e-9 * force);

    file = fullfile(out_dir, sprintf('%gN.csv', force));
    fid = fopen(file, 'w');
    fprintf(fid, 'r,A,Q,rho,mdot,v0,vi,v3,F,E\n');
    fprintf(fid, '%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g\n', data');
    fclose(fid);

    plot3(A_series, v3_series, data(:,10), 'LineWidth', 2);
end

grid on;
title('Ideal power for a given static thrust');
legend(arrayfun(@(f) sprintf('F = %g N', f), forces, 'UniformOutput', false));
xlabel('A [m^2]');
ylabel('v_3 [m/s]');
zlabel('P [W]');

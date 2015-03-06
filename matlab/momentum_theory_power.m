% fluid density
rho = 1.293; % [kg/m^3]

% induced velocity (Froude-Rankine theorem)
vi = @(v3,v0) 0.5*(v3+v0);

% volumentric flow rate
Q = @(A,vi) A*vi;

% mass flow rate
mdot = @(A,vi) rho*Q(A,vi);

% theoretical dynamic thrust
F = @(A,v3,v0) mdot(A,vi(v3,v0))*(v3-v0);

% theoretical energy required
E = @(A,v3,v0) F(A,v3,v0)*vi(v3,v0);

% outflow velocity from force and area
v3N = @(F,A) sqrt(2*F./(rho*A));

% area from outflow velocity and force
AN = @(F,v3) 2*F./(rho*v3.^2);

% fixed target force
force = 1; % [N]

% simulation
r_series = 0.01:0.001:0.1; % radius [m]
A_series = pi*r_series.^2;
v3_series_1 = v3N(0.5*force, A_series);    % outflow velocity [m/s]
v3_series_2 = v3N(force, A_series);    % outflow velocity [m/s]
v3_series_3 = v3N(2*force, A_series);    % outflow velocity [m/s]

energy_series_1 = nan(numel(A_series));
energy_series_2 = nan(numel(A_series));
energy_series_3 = nan(numel(A_series));
index = 0;
for A=A_series
    index = index + 1;         
    energy_series_1(index) = E(A,v3_series_1(index),0);
    energy_series_2(index) = E(A,v3_series_2(index),0);
    energy_series_3(index) = E(A,v3_series_3(index),0);
end


figure;
plot3(A_series,v3_series_1,energy_series_1, 'LineWidth', 2); hold on;
plot3(A_series,v3_series_2,energy_series_2, 'Color', 'r', 'LineWidth', 2);
plot3(A_series,v3_series_3,energy_series_3, 'Color', 'm', 'LineWidth', 2);


grid on;
title(['Kinetic energy for a static thrust of ~' num2str(force) ' N']);
xlabel('A [m^2]');
ylabel('v_3 [m/s]');
zlabel('E [J]');
legend('E_{0.5N}', 'E_{1N}', 'E_{2N}');
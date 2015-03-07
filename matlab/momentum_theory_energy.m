clear all;

% fluid density
rho = 1.293; % [kg/m^3]

% fixed target force
force = 8; % [N]

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

% simulation
r_min = 0.01;       % radius [m]
r_max = 0.1;        % radius [m]
N_points = 100;     % nunber of sampling points

r_series = linspace(r_min,r_max,N_points); % radius [m]
A_series = pi*r_series.^2;
v3_series = v3N(0.5*force, A_series);    % outflow velocity [m/s]
F_series = force * ones(size(A_series));
rho_series = rho * ones(size(A_series));

energy_series = nan(size(A_series));

data = [];

index = 0;
for A=A_series
    index = index + 1;

    v0 = 0;
    energy = E(A,v3_series(index),v0);
    energy_series(index) = energy;
    
    vi_ = vi(v3_series(index),v0);
    data(index,:) = [r_series(index), A, Q(A,vi_), rho_series(index), mdot(A,vi_), v0, vi_, v3_series(index), force, energy];
    
end

r = data(:,1);
A = data(:,2);
Q = data(:,3);
rho = data(:,4);
mdot = data(:,5);
v0 = data(:,6);
vi = data(:,7);
v3 = data(:,8);
F = data(:,9);
E = data(:,10);
T = table(r,A,Q,rho,mdot,v0,vi,v3,F,E);
writetable(T,'mt_energy.csv','Delimiter',',','WriteRowNames',true);

figure;
plot3(A_series,v3_series,energy_series, 'LineWidth', 2); hold on;

grid on;
title(['Kinetic energy for a static thrust of ~' num2str(force) ' N']);
xlabel('A [m^2]');
ylabel('v_3 [m/s]');
zlabel('E [J]');
function [K, cond_C] = place_ackermann(A, B, poles)
% PLACE_ACKERMANN  State feedback gain for a single-input system.
%
%   [K, cond_C] = place_ackermann(A, B, poles)
%
% Returns the row vector K for which the closed loop A - B K has the given
% eigenvalues (Ackermann's formula, K = [0 ... 0 1] C^-1 phi(A), with the
% controllability matrix C and the desired characteristic polynomial phi),
% and the condition number of C as a warning sign for numerical trouble.

n = size(A, 1);
C = zeros(n);
C(:, 1) = B;
for i = 2:n
    C(:, i) = A * C(:, i - 1);
end
cond_C = cond(C);
if rank(C) < n
    error('place_ackermann: (A, B) is not controllable');
end

% phi(A) by Horner's scheme; poly() gives real coefficients for conjugate pairs
c = real(poly(poles));
phi = zeros(n);
for i = 1:numel(c)
    phi = phi * A + c(i) * eye(n);
end

e = zeros(1, n);
e(n) = 1;
K = (e / C) * phi;
end

function [EN,F,M] = PARALLEL_PROPCALC(E,WP,R,C,TS)
% N
N = length(WP) / 3;

% Calculating the force vector
FX = zeros(1,N);
FY = zeros(1,N);
FZ = [-WP.^2 * C(1)]';
F = [FX; FY; FZ]';

% Calculating the moment created by the Aerodynamic drag
%   S is the sense
MD = [0, 0, -WP.^2 * C(2) * sign(WP)]';

% Total moment created about the center of mass
M = MD + cross(repmat(R,N,1), F);

% Wing orientation
EN = E + WP * TS;

EN = reshape(EN,3,[]);
F = reshape(F,3,[]);
M = reshape(M,3,[]);

end
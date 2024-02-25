function [EN,F,M] = PROPCALC(E,WP,R,C,S,TS)

% Calculating the force vector
F = [0, 0, -WP^2 * C(1)]';

% Calculating the moment created by the Aerodynamic drag
%   S is the sense
MD = [0, 0, -WP^2 * C(2) * S]';

% Total moment created about the center of mass
M = MD + cross(R, F);

% Wing orientation
EN = E + WP * TS;

end
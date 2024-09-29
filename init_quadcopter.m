% Simulation parameters
t_s = 0.01;
t_lim = 3.5;

% Quad-copter config
load quadcopter_database/tello.mat

% Rigid body parameters
m = quadcopter.m;
j = quadcopter.j;
j_i = quadcopter.j_i;

% Rigid body dimensions
dims = quadcopter.dims;

% Prop Locations
RP1 = [ dims(1)/2,  dims(2)/2, dims(3)/2]';
RP2 = [-dims(1)/2,  dims(2)/2, dims(3)/2]';
RP3 = [-dims(1)/2, -dims(2)/2, dims(3)/2]';
RP4 = [ dims(1)/2, -dims(2)/2, dims(3)/2]';

% Prop pose
E1 = 0;
E2 = 0;
E3 = 0;
E4 = 0;

% Prop coeffs
C = quadcopter.C;
C_CCW = [-C(1), C(2)];
C_CW  = [ C(1), C(2)];

% Allocation
X = zeros(3,t_lim/t_s);
E = zeros(3,t_lim/t_s);
V = zeros(3,t_lim/t_s);
W = zeros(3,t_lim/t_s);

% Initial conditions
X(:,1) = [2, 2, -2]';
E(:,1) = [0, 0, 0]';
V(:,1) = [0, 0, 0]';
W(:,1) = [0, 0, 0]';
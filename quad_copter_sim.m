%% Prep
% Use dynamics module
addpath('rigid_body_dynamics\')
addpath('quadcopter_database\')
addpath('functions\')

% Start
clc, clear, close all

% Simulation parameters
t_s = 0.01;
t_lim = 100;

% Quad-copter config
load tello.mat

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

%% Simulation
step_counter = 0;
for i = t_s:t_s:t_lim

    step_counter = step_counter + 1;

    err = -5 - X(3,step_counter);
    errd = V(3, step_counter);

    % Propeller angular velocities (CCW : - /// CW : +)
    WP1 =  1800 / 60 * (2 * pi) * err - 700 * errd;
    WP2 =  -2000 / 60 * (2 * pi) * err + 700 * errd;
    WP3 =  1800 / 60 * (2 * pi) * err - 700 * errd;
    WP4 =  -2000 / 60 * (2 * pi) * err + 700 * errd;

    [E1,F1,M1] = PROPCALC(E1,WP1,RP1,C_CCW,t_s);
    [E2,F2,M2] = PROPCALC(E2,WP2,RP2,C_CW ,t_s);
    [E3,F3,M3] = PROPCALC(E3,WP3,RP3,C_CCW,t_s);
    [E4,F4,M4] = PROPCALC(E4,WP4,RP4,C_CW ,t_s);
    
    F = F1 + F2 + F3 + F4 + 0 * CE2B(E(:,step_counter)) * [0, 0, 9.81]';
    M = M1 + M2 + M3 + M4;

    [X_NEW,E_NEW,V_NEW,W_NEW] = RK4UPDATESTATES(m,j,j_i,F,M,...
        X(:,step_counter),E(:,step_counter),V(:,step_counter), ...
        W(:,step_counter),t_s);

    X(:,step_counter + 1) = X_NEW;
    E(:,step_counter + 1) = ANGLEMAP(E_NEW);
    V(:,step_counter + 1) = V_NEW;
    W(:,step_counter + 1) = W_NEW;

    if mod(step_counter, 100) == 0
        PLOTOBJECT(3, 10*dims, X_NEW, E_NEW, 1)
    end
    
end

%% Plot
PLOTRESULTS([1,2],t_s,X,E,V,W)

%% Remove path
rmpath('rigid_body_dynamics\')
rmpath('quadcopter_database\')
rmpath('functions\')











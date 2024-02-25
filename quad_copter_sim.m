%% Prep
% Use dynamics module
addpath('rigid_body_dynamics\')

% Start
clc, clear, close all

% Simulation parameters
t_s = 0.001;
t_lim = 50;

% Rigid body parameters
m = 1;
j = 10 * eye(3);
j_i = inv(j);

% Rigid body dimensions
dims = [5 , 5, 0.5];

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
C = [1, 1];

% Allocation
X = zeros(3,t_lim/t_s);
E = zeros(3,t_lim/t_s);
V = zeros(3,t_lim/t_s);
W = zeros(3,t_lim/t_s);

% Initial conditions
X(:,1) = [2, 2, 2]';
E(:,1) = [0, 0, 0]';
V(:,1) = [0, 0, 0]';
W(:,1) = [0, 0, 0]';

%% Simulation
step_counter = 0;
for i = t_s:t_s:t_lim

    step_counter = step_counter + 1;

    WP1 = 1.01;
    WP2 = 1.01;
    WP3 = 1;
    WP4 = 1;

    [E1,F1,M1] = PROPCALC(E1,WP1,RP1,C, 1,t_s);
    [E2,F2,M2] = PROPCALC(E2,WP2,RP2,C,-1,t_s);
    [E3,F3,M3] = PROPCALC(E3,WP3,RP3,C, 1,t_s);
    [E4,F4,M4] = PROPCALC(E4,WP4,RP4,C,-1,t_s);
    
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
        PLOTOBJECT(3, dims, X_NEW, E_NEW, 1)
        % pause(0.1)
    end
    
end

%% Plot
PLOTRESULTS([1,2],t_s,X,E,V,W)

%% Remove path
rmpath('rigid_body_dynamics\')













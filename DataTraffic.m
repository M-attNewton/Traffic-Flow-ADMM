%% Select default or custom data
default = 0;

if default == 0
%% Autonomous vehicle data
AV_number = 1; % number of autonomous vehicles
N = 5; % Number of vehicles
CR = 1;  %communication range
% Default values
% AV_number = 1; % number of autonomous vehicles
% N = 20; % Number of vehicles
% CR = 9;  %communication range

%% Vehicle dynamics data
L = 100; %ring length
v_star = 15; %desired velocity
s_star = L/N; %desired spacing
accel_min = -5;
accel_max = 2;
alpha  = 0.3 + (-0.1 + 0.2.*rand(N,1)); %sensitivity coeff
beta   = 0.3 + (-0.1 + 0.2.*rand(N,1)); %sensitivity coeff
v_max  = 30; %max velocity
s_st   = 5; %spacing parameters
s_go   = 35 + (-5 + 10.*rand(N,1));

% Control parameters
s_star = L/N;
gamma_s = 0.03;
gamma_v = 0.15;
gamma_u = 1;


%% Controller pattern
K_Pattern = Pattern_Generation(N,AV_number,CR);

%% Graphs
% Plant graph, only the vehicle in front has influence on the ith vehicle
Gp = zeros(N,N);
for i = 1:N-1
    Gp(i,i+1) = 1;
end
Gp(N,1) = 1;

% Communication graph
Gc = zeros(N,N);
for i = 1:CR
    Gc(i,N) = 1;
    Gc(N-i,N) = 1;
end
Gc(N,N) = 1;

%Gc = [1 1 0;
 %     1 1 1;
  %    0 1 1];
  
% Put into structure to make functions easier  
params.alpha = alpha;
params.beta = beta;
params.v_max = v_max;
params.s_st = s_st;
params.s_go = s_go; 

gammas.s = gamma_s;
gammas.v = gamma_v;
gammas.u = gamma_u;




%% Default data
elseif default == 1
%% Autonomous vehicle data
AV_number = 1; % number of autonomous vehicles
N = 20; % Number of vehicles
CR = 5;  %communication range
% Default values
% AV_number = 1; % number of autonomous vehicles
% N = 20; % Number of vehicles
% CR = 9;  %communication range

%% Vehicle dynamics data
L = 400; %ring length
v_star = 15; %desired velocity
s_star = L/N; %desired spacing
accel_min = -5;
accel_max = 2;
alpha  = 0.6 + (-0.1 + 0.2.*rand(N,1)); %sensitivity coeff
beta   = 0.9 + (-0.1 + 0.2.*rand(N,1)); %sensitivity coeff
v_max  = 30; %max velocity
s_st   = 5; %spacing parameters
s_go   = 35 + (-5 + 10.*rand(N,1));

% Control parameters
s_star = L/N;
gamma_s = 0.03;
gamma_v = 0.15;
gamma_u = 1;


%% Controller pattern
K_Pattern = Pattern_Generation(N,AV_number,CR);

%% Graphs
% Plant graph, only the vehicle in front has influence on the ith vehicle
Gp = zeros(N,N);
for i = 1:N-1
    Gp(i,i+1) = 1;
end
Gp(N,1) = 1;

% Communication graph
Gc = zeros(N,N);
for i = 1:CR
    Gc(i,N) = 1;
    Gc(N-i,N) = 1;
end
Gc(N,N) = 1;

%Gc = [1 1 0;
 %     1 1 1;
  %    0 1 1];
  
% Put into structure to make functions easier  
params.alpha = alpha;
params.beta = beta;
params.v_max = v_max;
params.s_st = s_st;
params.s_go = s_go; 

gammas.s = gamma_s;
gammas.v = gamma_v;
gammas.u = gamma_u;

end

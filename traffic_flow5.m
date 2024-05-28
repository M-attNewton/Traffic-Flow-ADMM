%% Implementing the clique solving ADMM algorithm for traffic flow
% Notation will be consistend with the paper Controllability Analysis and Optimal Control of
% Mixed Traffic Flow with Human-driven and Autonomous Vehicles
clc; clear; close all

clear yalmip
%% Define parameters
% All data
DataTraffic

%% Solve using ADMM algorithm


% Create state space matricies
[A,B,H,Q,R,A_hat] = traffic_model(N,s_star,AV_number,gammas,params);

Q = Q*100;
R = R*10;

R1 = 10;


% Centralised algortihm (for comparison)
[Kc,Xc] = centralised1(A,B,H,Q,R1);

% Remove zero eigenvalues
[A] = remove_eigenvalues(A);

% Decentralised controller
[Kd,Xd,h2] = decentralised2(A,B,H,Q,R1,K_Pattern);
%[Kdc,Xdc] = decentralised_centralised2(A,B,H,Q,R,K_Pattern);

% Mirror graph and super graph
G = (Gc+Gc') + (Gp+Gp'); 

% Dentralised AMDD algorithm
[K] = decentralised_ADMM2(G,A,B,H,Q,R);
K = sum(K,1);

% Turn off controller
% K = zeros(1,2*N);
% A = A_hat;

% Old functions
%[K] = decentralised_centralised1(A,B,H,Q,R,K_Pattern);
%[Kd,Xd] = decentralised1(A,B,H,Q,R);


%% Simulate driving
% Simluation data
num_steps = 50000;
time_step = 0.01;

% Intial conditions
p = [L/N:L/N:L]';
s = -L/N*ones(N,1); 
v = -(15 + (-4 + 8.*rand(N,1)));
%time = 0:time_step:time_step*num_steps;

% Simluate traffic flow for each time step
[x,x_dot,p,s,v] = simulate_traffic(time_step,num_steps,N,p,s,v,A,B,K,accel_max,accel_min,s_star,v_star);

% Plots the traffic dynamics
plot_traffic(L,num_steps,N,p,s,v,time_step)

% Plots the velocity of the vehicles
plot_velocity(num_steps,time_step,v)



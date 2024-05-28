% Generate a dynamic model for ring of vehicles
function [A,B,H,Q,R,A_hat] = traffic_model(N,s_star,AV_number,gammas,params)

% Unpack parameters
alpha = params.alpha;
beta = params.beta;
v_max = params.v_max;
s_st = params.s_st;
s_go = params.s_go;

gamma_s = gammas.s;
gamma_v = gammas.v;
gamma_u = gammas.u;

A1 = zeros(2,2,N);
A2 = zeros(2,2,N);
for i = 1:N

    % Define alpha terms for dynamic model
    alpha1 = alpha(i)*v_max/2*pi/(s_go(i)-s_st)*sin(pi*(s_star-s_st)/(s_go(i)-s_st));
    alpha2 = alpha(i) + beta(i);
    alpha3 = beta(i);

    A1(:,:,i) = [0,-1;alpha1,-alpha2];
    A2(:,:,i) = [0,1;0,alpha3];
    C1 = [0,-1;0,0];
    C2 = [0,1;0,0];

end

% Indexing
pos1 = 1;
pos2 = N;

A = zeros(2*N,2*N);
for i = 1:(N-1)
    A((2*i-1):(2*i),(2*pos1-1):(2*pos1)) = A1(:,:,i);
    A((2*i-1):(2*i),(2*pos2-1):(2*pos2)) = A2(:,:,i);
    pos1 = pos1+1;
    pos2 = mod(pos2+1,N);
end

% Controllerable parts
A((2*N-1):(2*N),(2*pos1-1):(2*pos1)) = C1;
A((2*N-1):(2*N),(2*pos2-1):(2*pos2)) = C2;

% Turn off controller
A_hat = A;
A_hat((2*N-1):(2*N),(2*pos1-1):(2*pos1)) = A1(:,:,N);
A_hat((2*N-1):(2*N),(2*pos2-1):(2*pos2)) = A2(:,:,N);

% Controller matrices
Q = zeros(2*N);
for i = 1:N
    Q(2*i-1,2*i-1) = gamma_s;
    Q(2*i,2*i) = gamma_v;
end

B = zeros(2*N,AV_number);
B(2*N,AV_number) = 1;
if AV_number == 2
    AV2_Index = floor(N/2);
    A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-1):(2*AV2_Index))=C1;
    A((2*AV2_Index-1):(2*AV2_Index),(2*AV2_Index-3):(2*AV2_Index-2))=C2;
    B(2*AV2_Index,1) = 1;
end

H = zeros(2*N,N);
for i=1:N
   H(2*i,i) = 1; 
end

% Flatten disturbance matrix
%H = sum(H,2);

R = gamma_u*eye(AV_number,AV_number);
R = gamma_u*eye(N,N);


    
end


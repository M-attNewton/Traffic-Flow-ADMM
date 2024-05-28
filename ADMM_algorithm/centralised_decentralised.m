function [K,Cost, info] = centralised_decentralised(G,A,B,H,Q,R,usersflag)
% Centralized Design of decentralized optimal controller
% dot(x) = Ax + Bu + Md
% u = Kx, K is block diagonal
% Q,R cell structure, performance index

if(nargin >= 7)
    Flag = usersflag;
else
    Flag = 1;
end

N = size(A,2)/2;

% Convert matrix to cell 

% A = mat2cell(A, [2*ones(N,1)], [2*ones(N,1)]);
% B = mat2cell(B, [2*ones(N,1)], [1]);
% M = mat2cell(M, [2*ones(N,1)], [1]);
% Q = mat2cell(Q, [2*ones(N,1)], [2*ones(N,1)]);
% R = mat2cell(R, [ones(N,1)], [ones(N,1)]);

opts.subbose = true;
epsilon      = 1e-2;

cvx_begin sdp

variable X(2*N,2*N)
variable Y(1,1) 
variable Z(1,2*N)

minimize( trace(Q*X) + trace(R*Y) )

subject to

(A*X - B*Z) + (A*X - B*Z)' + H*H' <= 0;
%A*X + X*A' - B*Z - Z'*B' + H*H' <= 0;

[Y, Z; Z', X] >= 0;

X >= 0 ;

cvx_end

K = Z/X; % Z*inv(X);
Cost = 1;
info = 1;

% 
% 
% tic
% n            = size(G,1);            % Dimension number of nodes
% subDimension  = zeros(n,1);          % state dimension
% subDimensionI = zeros(n,1);          % input dimension
% subDimensiond = zeros(n,1);          % input dimension
% for i = 1:n
%     subDimension(i) = size(A{i,i},1);
%     subDimensionI(i) = size(B{i},2);
%     subDimensiond(i) = size(M{i},2);
% end
% 
% % global form
% accDimen  = [cumsum([1;subDimension])]; 
% gA   = zeros(sum(subDimension));  % gloabl state space model
% X         = [];
% Y         = [];
% Z         = [];  
% gB   = [];
% gM   = [];
% gQ = [];
% gR = [];
% %globalP   = zeros(sum(subDimension)); 
% for i = 1:n
%     X = blkdiag(X,sdpvar(subDimension(i)));
%     Y = blkdiag(Y,sdpvar(subDimensionI(i)));
%     Z = blkdiag(Z,sdpvar(subDimensionI(i),subDimension(i)));
%     gB = blkdiag(gB,B{i});
%     gM = blkdiag(gM,M{i});
%     gQ = blkdiag(gQ,Q{i});
%     gR = blkdiag(gR,R{i});
%     gR = R{1,1};
%     for j = 1:n
%         if G(i,j) ~= 0 || i == j
%             gA(accDimen(i):accDimen(i+1)-1,accDimen(j):accDimen(j+1)-1) = A{i,j};
%         end
%     end
% end
% 
% %Y = sdpvar(sum(subDimensionI));
% 
% 
% %% define Cost and constraints
% %gB = sum(gB,2);
% Constraints = [(gA*X - gB*Z) + (gA*X - gB*Z)'+ gM*gM' <=0];
% Constraints = [Constraints, X - epsilon*eye(sum(subDimension)) >=0];
% 
% Constraints = [Constraints, [Y Z;Z' X] >= 0];
% 
% Cost = trace(gQ*X) + trace(gR*Y);
% 
% if Flag == 1  % use SeDuMi
%     options = sdpsettings('verbose',opts.subbose,'solver','sedumi');
%     sol     = optimize(Constraints,Cost,options);
%     
%     %model = export(Constraints,Cost,options);
% elseif Flag == 2  % SparseCoLO
%     opts          = sdpsettings('verbose',1,'solver','sparsecolo','sparsecolo.SDPsolver','sedumi','sparsecolo.domain',1,'sparsecolo.range',0,'sparsecolo.EQorLMI',1);
%     sol     = optimize(Constraints,Cost,opts);
% elseif Flag == 3  % CDCS
%     options = sdpsettings('verbose',opts.subbose,'solver','cdcs');
%     sol     = optimize(Constraints,Cost,options);
% end
% 
% timeTotal    = toc;
% info.time    = [timeTotal,sol.solvertime]; 
% info.gA = gA;
% info.gB = gB;
% info.gM = gM;
% info.X = value(X);
% info.Y = value(Y);
% info.Z = value(Z);
% 
% %% set values
% X = value(X);
% Cost = value(Cost);
% K = value(Z)*X^(-1);
% 
% ClosedSys = ss(gA - gB*K,gM,[gQ^(1/2);gR^(1/2)*K],[]);
% info.h2  = norm(ClosedSys,2);

end


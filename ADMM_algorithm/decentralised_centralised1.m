function [K] = decentralised_centralised1(A,B,H,Q,R,K_Pattern)
% Dentralized Design of optimal controller in centralised way
% dot(x) = Ax + Bu + Hd
% u = Kx, 
% Q,R cell structure, performance index

N = size(A,2)/2;

% Find sparsity pattern matrix
Tp = K_Pattern;
Sp = patterninvariance(K_Pattern);

%cvx_solver mosek
%cvx_save_prefs

% Optimal control problem, minimising H2 norm
cvx_begin sdp

variable X(2*N,2*N)
variable Y(1,1) 
variable Z(1,2*N)

minimize( trace(Q*X) + trace(R*Y) )

subject to

(A*X - B*Z) + (A*X - B*Z)' + H*H' <= 0;

[Y, Z; Z', X] >= 0;

X >= 0 ;

% Sparsity constraints
for i = 1:size(Sp,1)
    for j = 1:size(Sp,2)
       if Sp(i,j) == 0
           X(i,j) == 0; 
           X(j,i) == 0;
       end
    end
end
        
for i = 1:size(Tp,1)
    for j = 1:size(Tp,2)
       if Tp(i,j) == 0
           Z(i,j) == 0;
       end
    end
end

cvx_end

K = Z/X; 

end


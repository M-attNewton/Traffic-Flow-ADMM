function [K,X,h2] = decentralised2(A,B,H,Q,R,K_Pattern)
% Centralized Design of optimal controller
% dot(x) = Ax + Bu + Hd
% u = Kx, 
% Q,R cell structure, performance index

N = size(A,2)/2;

% Find sparsity pattern matrix
Tp = K_Pattern;
Sp = patterninvariance(K_Pattern);

cvx_solver sedumi 
cvx_save_prefs

cvx_begin sdp

variable Xi(2,2*N)    % block diagonal structure
variable Y(1,1) 
variable Z(1,2*N)

X = Xi(:,1:2);
for i = 2:N
    X = blkdiag(X,Xi(:,2*(i-1)+1:2*i));   % block diagonal structure
end

minimize( trace(Q*X) + trace(R*Y) )

subject to

    (A*X - B*Z) + (A*X - B*Z)' + H*H' <= 0;

    [Y, Z; Z', X] >= 0;

    X >= 0 ;
    
    for i = 1:size(Tp,1)
        for j = 1:size(Tp,2)
           if Tp(i,j) == 0
               Z(i,j) == 0;
           end
        end
    end
     
% for i = 1:2*(N-1)
%     Z(1,i) == 0
% end

% for i = 1:size(Sp,1)
%     for j = 1:size(Sp,2)
%        if Sp(i,j) == 0
%            X(i,j) == 0; 
%            X(j,i) == 0;
%        end
%     end
% end
    
cvx_end

K = Z/X; 

ClosedSys = ss(A - B*K,H,[Q^(1/2);R^(1/2)*K],[]);
    h2  = norm(ClosedSys,2);


end
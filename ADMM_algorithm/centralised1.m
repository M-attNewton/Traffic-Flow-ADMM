function [K] = centralised1(A,B,H,Q,R)
% Centralized Design of optimal controller
% dot(x) = Ax + Bu + Hd
% u = Kx, 
% Q,R cell structure, performance index

N = size(A,2)/2;

%cvx_solver mosek
%cvx_save_prefs

cvx_begin sdp

variable X(2*N,2*N)
variable Y(1,1) 
variable Z(1,2*N)

minimize( trace(Q*X) + trace(R*Y) )

subject to

(A*X - B*Z) + (A*X - B*Z)' + H*H' <= 0;

[Y, Z; Z', X] >= 0;

X >= 0 ;

cvx_end

K = Z/X; 

end


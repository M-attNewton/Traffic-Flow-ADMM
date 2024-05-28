
%% test 

close all
clear yalmip

Gp = [1 0 0 0; 1 1 0 0;0 1 1 1;1 1 0 1];

A = cell(4,4);
 A{1,1} = 1; A{2,1} = 1; A{2,2} = 2;
 A{3,2} = 2; A{3,3} = 3; A{3,4} = 4;
 A{4,1} = 1; A{4,2} = 2; A{4,4} = 4;

B = cell(4,1);
B{1} = 1; B{2} = 1; B{3} = 1; B{4} = 1;


M = B;
Q = B;
R = B;
% 2 not good
% 5 looks ok
% 10,15 looks ok+1
% 20, 30 not ok
% 50, 55, 60 ok
% 80/85/100--> best or 70
opts.mu = 10;%500;
[K, Cost, info] = decentralised_ADMM2(Gp,A,B,M,Q,R,opts);

%[K0, Cost0, info0] = cdd(Gp,A,B,M,Q,R);

% 
% close all
% range1 = 1:55;%20:80;
% figure
% semilogy(info.presi(range1));
% 
% figure
% range2 = 1:info.iter;%20:80;
% semilogy(info.dresi(range1))
% figure
% semilogy((abs(info.cost(range1) - Cost0)./Cost0))


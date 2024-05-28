%% Implementing the clique solving ADMM algorithm for traffic flow
% Notation will be consistend with the paper Controllability Analysis and Optimal Control of
% Mixed Traffic Flow with Human-driven and Autonomous Vehicles
clc; clear; close all

%% Define parameters
% All data
DataTraffic

%% Solve using ADMM algorithm

% Optimisation parameters
% opts.eps = 1.0e-6;
% opts.maxIter = 250;
% opts.mu  = 500;

% Create state space matricies
%[A,B,H,Q,R] = RingMixedModel3(N,s_star,gamma_s,gamma_v,gamma_u,AV_number);   % Dynamics & Performance index
[A,B,H,Q,R,A_hat] = traffic_model(N,s_star,AV_number,gammas,params);   

% ADMM algorithm
%[K,Cost,info] = ADMM2(Gp,A,B,H,Q,R,opts);

% Centralised algortihm (for comparison)
[Kc] = centralised1(A,B,H,Q,R);


%% added by Yang, here I tried to solve the decentralized version in a centralized way
%  where I assume a block-diagonal Lyapunov function. This leads to an
%  infeasible problem, which explains the behavior of ADMM algorithm. 
Kd   = decentralised1(A,B,H,Q,R);

% Remove zero eigenvalues
% [vec,eigen] = eig(A);
% for i = 1:size(eigen,1)
%     if abs(eigen(i,i)) <= 10^-10
%         A = A - vec(:,i)*vec(:,i)'/norm(vec(:,i),2);
%     end
% end

% Denctralised control in centralised way
%[K] = decentralised_centralised1(A,B,H,Q,R,K_Pattern);

% Mirror graph and super graph
G = (Gc+Gc') + (Gp+Gp'); 

% Dentralised AMDD algorithm
[K] = decentralised_ADMM2(G,A,B,H,Q,R);
K = sum(K,1);

% Turn off controller
%K = zeros(1,2*N);
%A = A_hat;


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
figure
for t = 1:10:num_steps
    plot(v(:,1:t)')
    drawnow
    pause on
    pause(time_step*0.1)
end



%% Junk


% % Iterate each time step for each vehicle
% for t = 1:num_steps
%     for i = [N, 1:N-1]
%         
%         % Error states for control model
%         s_tilda(i,t) = s(i,t) + s_star;
%         v_tilda(i,t) = v(i,t) + v_star;
%         
%         x(2*i-1,t) = s_tilda(i,t); 
%         x(2*i,t) = v_tilda(i,t);
%                
%     end  
%     
%     % State space model
%     x_dot(:,t) = A*x(:,t) - B.*K'.*x(:,t);
%     
%     % Follow stopper algorithm
%     %[x_dot] = follower_stopper(i,N,v,s,x_dot);
% 
%     % Limit max and min acceleration
%     for i = 1:N
%         if x_dot(2*i,t) > accel_max
%             x_dot(2*i,t) = accel_max;
%         end
%         if x_dot(2*i,t) < accel_min
%             x_dot(2*i,t) = accel_min;
%         end
%         if i == 1
%             if abs((v(i,t))^2 - (v(N,t))^2)/(2*s(i,t)) >= abs(accel_min)
%                 x_dot(2*i,t) = accel_min;
%             end
%         else
%             if abs((v(i,t))^2 - (v(i-1,t))^2)/(2*s(i,t)) >= abs(accel_min)
%                 x_dot(2*i,t) = accel_min;
%             end
%         end
%         
%     end
%     
%     % Update state space using forward difference
%     x(:,t+1) = x_dot(:,t)*time_step + x(:,t);
%     
%     for i = 1:N
%         
%         s(i,t+1) = x(2*i-1,t+1) - s_star;
%         v(i,t+1) = x(2*i,t+1) - v_star;
%         
%         % Approximate position for now
%         p(i,t+1) = p(i,t) + time_step*(v(i,t) + v(i,t+1))/2;
% 
%         % Solve positions from s terms       
%         if i < N
%             mat_temp(i,i) = -1;
%         end
%         if i < N-1
%             mat_temp(i+1,i) = 1;
%         end
%       
%     end
%     
%     % Approximate position of first vehicle
%     p(1,t+1) = p(1,t) + time_step*(v(1,t) + v(1,t+1))/2;
%     
% end

%     if i == N
%     delta_x1 = 12.5;
%     delta_x2 = 14.75;
%     delta_x3 = 20;
%     
%     if i == N
%         v_lead = v(1,t);
%     else
%         v_lead = v(i+1,t);
%     end
%     
%     U_ = (1/t)*sum(v(i,:));
%     
%     v_ = min(max(v_lead, 0),U_);
%     
%     delta_x = abs(s(i,t));
%     if abs(s(i,t)) <= delta_x1
%         v_cmd = 0;
%     elseif delta_x1 <= abs(s(i,t)) && abs(s(i,t)) <= delta_x2
%         v_cmd = v_*(delta_x - delta_x1)/(delta_x2 - delta_x1);
%     elseif delta_x2 <= abs(s(i,t)) && abs(s(i,t)) <= delta_x3
%         v_cmd = v_ + (U_ - v_)*(delta_x - delta_x2)/(delta_x3 - delta_x2);
%     elseif delta_x3 <= abs(s(i,t))
%         v = U_;
%     end
%     
%     g1 = 7;
%     g_u = 30;
%     v_catch = 1;
%     
%     v_target = U_ + v_catch*min(max((delta_x - g1)/(g_u - g1),0),1);
%     
%     alpha = 0.6;
%     beta = 0.9;
%     v_cmd2 = beta*(alpha*v_target + (1-alpha)*v_lead) + (1-beta)*v_cmd;
%     
%     x_dot(N,t) = -alpha*(v_cmd2 - v(N,t));
%     end
    


% figure
% colour = rand(N,3);
% r = L/(2*pi);
% for t = 1:10:num_steps
%     for i = 1:N
%         
%         % Find polar cooridates 
%         theta = (2*pi/L)*p(i,t);
% 
%         % Plot positions in polar plot
%         polarplot(theta,r,'o','MarkerSize',10,'MarkerFaceColor',colour(i,:),'MarkerEdgeColor',colour(i,:));
%         ax = gca;
%         ax.ThetaGrid = 'off';
%         ax.RGrid = 'off';
%         ax.RTickLabel = []; 
%         ax.ThetaTickLabel = [];
%         hold on
%         
%     end 
%     pause on
%     pause(time_step*0.1)
%     hold off
% end

   % p(2:end,t+1) = mat_temp\s(2:end,t+1) + [p(1,t+1);zeros(N-2,1)];
    
%         x_temp = [s_tilda(:,t); v_tilda(:,t)];
%         x = x_temp(:);
% 
%         % Temp solution
%         x = (A - sum(K',2))*x;
%         s(i,t+1) = x(2*i-1);
%         v(i,t+1) = x(2*i);

%         if i == 1
%             if abs(p(N,t) - p(1,t)) > 350
%                 s(i,t) = p(N,t) - p(1,t) - L;            
%             else
%                 s(i,t) = p(N,t) - p(1,t);    
%             end
% 
%             %s_dot(i,t) = v(N,t) - v(1,t);
%         else
%             s(i,t) = p(i-1,t) - p(i,t);
%             %s_dot(i,t) = v(i-1,t) - v(i,t);
%         end


%plot_dynamics(p,s,v,v_dot);

% function [V] = velocity_func(s,s_st,s_go,v_max)
% 
% if abs(s) <= s_st
%     V = 0;
% elseif abs(s) > s_st && abs(s) < s_go
%     V = v_max*0.5*(1 - cos(pi*(abs(s) - s_st)/(s_go - s_st)));
% elseif abs(s) > s_go
%     V = v_max;
% end
% 
% end

%function [accel] = accel_func()
% 
% function plot_dynamics(p,s,v,v_dot)
% figure
% 
% for t = 1:num_steps
%     for i = 1:N
%         
%         % Find polar cooridates 
%         theta = 2*pi/L*p(i,t);
%         r = L/(2*pi);
%         
%         % Find x,y coordinates
%         %x = L/(2*pi) * cos(2*pi*p(i,t)/L);
%         x = r*cos(theta*p(i,t));
%         y = r*sin(theta*p(i,t));
%         
%         plot(x,y);
%         
%     end 
% end
% 
% end


% 
% for t = 1:num_steps
%     for i = [N, 1:N-1] % Need to compute N first to avoid errors
%     % Compute all values for time t
%     if i == 1
%         if abs(p(N,t) - p(1,t)) > 350
%             s(i,t) = p(N,t) - p(1,t) - L;
%         else
%             s(i,t) = p(N,t) - p(1,t);
%         end
%             
%         s_dot(i,t) = v(N,t) - v(1,t);
%     else
%         s(i,t) = p(i-1,t) - p(i,t);
%         s_dot(i,t) = v(i-1,t) - v(i,t);
%     end
%     
%     % Calculate acceleration
%     v_dot(i,t) = alpha(i)*(velocity_func(s(i,t),s_st,s_go(i),v_max) - v(i,t)) + beta(i)*s_dot(i,t);
%     
%     if  i == 1
%         %v_dot(N,t) = alpha(N)*(velocity_func(s(N,t),s_st,s_go(N),v_max) - v(N,t)) + beta(N)*s_dot(N,t);
%         if abs(v_dot(i,t)^2 - v_dot(N,t)^2)/(2*abs(s(i,t))) >= abs(accel_min)
%            % v_dot(N,t) = accel_min;
%         end
%         if v_dot(i,t) <= accel_min
%             v_dot(i,t) = accel_min;
%         elseif v_dot(i,t) > accel_max
%             v_dot(i,t) = accel_max;
%         end
%         
%     else
%     
%         if abs(v_dot(i,t)^2 - v_dot(i-1,t)^2)/(2*abs(s(i,t))) >= abs(accel_min)
%            % v_dot(i-1,t) = accel_min;
%         end
%         if v_dot(i,t) <= accel_min
%             v_dot(i,t) = accel_min;
%         elseif v_dot(i,t) > accel_max
%             v_dot(i,t) = accel_max;
%         end
%     
%     end
%     
%          % Update position and velocity
%         p(i,t+1) = p(i,t) + v(i,t)*time_step;
%         v(i,t+1) = abs((v(i,t)^2 + 2*v_dot(i,t)*(p(i,t+1) - p(i,t))))^0.5;
%     
%     
% %     % Find values for CAV using controller
% %     if i == N
% %         x = [s(:,t); v(:,t)];
% %         x = x(:);
% % 
% %         % Temp solution
% %         x = (A - sum(K',2))*x;
% %         s(i,t+1) = x(2*i-1);
% %         v(i,t+1) = x(2*i);
% %         
% %     else 
% %         % Update position
% %         p(i,t+1) = p(i,t) + v(i,t)*t;
% %         v(i,t+1) = abs((v(i,t)^2 + 2*v_dot(i,t)*(p(i,t+1) - p(i,t))))^0.5;
% %         
% %     end
%     
%     
%     end
% end


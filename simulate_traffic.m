function [x,x_dot,p,s,v] = simulate_traffic(time_step,num_steps,N,p,s,v,A,B,K,accel_max,accel_min,s_star,v_star)


% Iterate each time step for each vehicle
for t = 1:num_steps
    for i = [N, 1:N-1]
        
        % Error states for control model
        s_tilda(i,t) = s(i,t) + s_star;
        v_tilda(i,t) = v(i,t) + v_star;
        
        x(2*i-1,t) = s_tilda(i,t); 
        x(2*i,t) = v_tilda(i,t);
               
    end  
    
    % State space model
    x_dot(:,t) = A*x(:,t) - B.*K'.*x(:,t);
    
    % Follow stopper algorithm
    %[x_dot] = follower_stopper(i,N,v,s,x_dot);

    % Limit max and min acceleration
    for i = 1:N
        if x_dot(2*i,t) > accel_max
            x_dot(2*i,t) = accel_max;
        end
        if x_dot(2*i,t) < accel_min
            x_dot(2*i,t) = accel_min;
        end
        if i == 1
            if abs((v(i,t))^2 - (v(N,t))^2)/(2*s(i,t)) >= abs(accel_min)
                x_dot(2*i,t) = accel_min;
            end
        else
            if abs((v(i,t))^2 - (v(i-1,t))^2)/(2*s(i,t)) >= abs(accel_min)
                x_dot(2*i,t) = accel_min;
            end
        end
        
    end
    
    % Update state space using forward difference
    x(:,t+1) = x_dot(:,t)*time_step + x(:,t);
    
    for i = 1:N
        
        s(i,t+1) = x(2*i-1,t+1) - s_star;
        v(i,t+1) = x(2*i,t+1) - v_star;
        
        % Approximate position for now
        p(i,t+1) = p(i,t) + time_step*(v(i,t) + v(i,t+1))/2;

        % Solve positions from s terms       
        if i < N
            mat_temp(i,i) = -1;
        end
        if i < N-1
            mat_temp(i+1,i) = 1;
        end
      
    end
    
    % Approximate position of first vehicle
    p(1,t+1) = p(1,t) + time_step*(v(1,t) + v(1,t+1))/2;
    
end
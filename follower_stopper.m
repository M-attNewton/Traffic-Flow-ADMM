function [x_dot] = follower_stopper(i,N,v,s,x_dot)


    % Follow stopper algorithm
    if i == N
    delta_x1 = 12.5;
    delta_x2 = 14.75;
    delta_x3 = 20;
    
    if i == N
        v_lead = v(1,t);
    else
        v_lead = v(i+1,t);
    end
    
    U_ = (1/t)*sum(v(i,:));
    
    v_ = min(max(v_lead, 0),U_);
    
    delta_x = abs(s(i,t));
    if abs(s(i,t)) <= delta_x1
        v_cmd = 0;
    elseif delta_x1 <= abs(s(i,t)) && abs(s(i,t)) <= delta_x2
        v_cmd = v_*(delta_x - delta_x1)/(delta_x2 - delta_x1);
    elseif delta_x2 <= abs(s(i,t)) && abs(s(i,t)) <= delta_x3
        v_cmd = v_ + (U_ - v_)*(delta_x - delta_x2)/(delta_x3 - delta_x2);
    elseif delta_x3 <= abs(s(i,t))
        v = U_;
    end
    
    g1 = 7;
    g_u = 30;
    v_catch = 1;
    
    v_target = U_ + v_catch*min(max((delta_x - g1)/(g_u - g1),0),1);
    
    alpha = 0.6;
    beta = 0.9;
    v_cmd2 = beta*(alpha*v_target + (1-alpha)*v_lead) + (1-beta)*v_cmd;
    
    x_dot(N,t) = -alpha*(v_cmd2 - v(N,t));
    end
    
end
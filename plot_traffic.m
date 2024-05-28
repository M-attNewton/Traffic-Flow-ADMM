function plot_traffic(L,num_steps,N,p,s,v,time_step)

figure
colour = rand(N,3);
colour(N,:) = [1,0,0];
r = L/(2*pi);
for t = 1:2:num_steps
    for i = 1:N
        
        % Find polar cooridates 
        theta = (2*pi/L)*p(i,t);

        % Plot positions in polar plot
        if i == N
            polarplot(theta,r,'o','MarkerSize',20,'MarkerFaceColor',colour(N,:),'MarkerEdgeColor',[0, 0, 0]);
        else
            polarplot(theta,r,'o','MarkerSize',10,'MarkerFaceColor',colour(i,:),'MarkerEdgeColor',colour(i,:));
        end
        ax = gca;
        ax.ThetaGrid = 'off';
        ax.RGrid = 'off';
        ax.RTickLabel = []; 
        ax.ThetaTickLabel = [];
        hold on
        
    end 
    pause on
    pause(time_step*0.1)
    hold off  
end


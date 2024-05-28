function plot_velocity(num_steps,time_step,v)
figure
for t = 1:10:num_steps
    plot(v(:,1:t)')
    drawnow
    pause on
    pause(time_step*0.1)
end
end
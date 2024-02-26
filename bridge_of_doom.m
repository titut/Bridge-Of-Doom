addpath("Neato\")

clear;
syms u;

%Creating R
speed = 0.11;
ri = 0.3960 * cos(2.65 * (speed*u + 1.4));
rj = -0.99 * sin(speed*u + 1.4);
r = [ri, rj, 0];

%Calculating Tangent Vector and Speed
drdu = diff(r, u);
dsdt = norm(drdu);

%Tangent Unit Vector
t_hat = drdu/norm(drdu);

%Calculating Normal Vector
t_hat_dot = diff(t_hat, u);
n_hat = t_hat_dot/norm(t_hat_dot);

%Calculating Angular Speed
omega = cross(t_hat, t_hat_dot);
omega = omega(3);

%Calculating left and right wheel speeds
d=0.245;
v_r = (2 * dsdt + omega * d)/2;
v_l = (2 * dsdt) - v_r;

%{

%Parametric curve plot
plot(x,y); hold on;

clf;

for i = 1:3
    time = i * 10;
    start = [subs(ri, u, time) subs(rj, u, time)];
    t_hat_at_time = subs(t_hat, u, time);
    n_hat_at_time = subs(n_hat, u, time);
    x_t_vec = [start(1) start(1)+t_hat_at_time(1)];
    y_t_vec = [start(2) start(2)+t_hat_at_time(2)];
    x_n_vec = [start(1) start(1)+n_hat_at_time(1)];
    y_n_vec = [start(2) start(2)+n_hat_at_time(2)];
    
    plot(x_t_vec, y_t_vec);
    plot(x_n_vec, y_n_vec);

    vpa(subs(omega, u, time))
end
hold off;

clf;

% Linear speed and Angular Velocity plot
subplot(2, 1, 1);
fplot(dsdt, [0, 32]);
subplot(2, 1, 2);
fplot(omega, [0, 32]);

clf;

% Wheel Velocity Plot

subplot(2, 1, 1);
fplot(v_l, [0 32]);
subplot(2, 1, 2);
fplot(v_r, [0 32]);

%}

%Connect to your Neato or the Simulator - choose one or the other
%[sensors,vels]=neatoSim(); %uncomment for simulator
[sensors,vels]=neato('192.168.16.99') %uncomment for physical neato
fig = gcf;
disp('press enter to continue');
pause;

%Matrix to store data
max_points = 10;
encoder_data=zeros(max_points,3);
iter=0;

%Estimated time for Neato to complete obstacle course
drivetime = (1/speed) * 3.2;

tic %%start your timer in Matlab
t=toc; %initiate t as the time since you started
while t<drivetime
    %Store encoder data into matrix
    t=toc; %t update t
    iter=iter+1;
    encoder_data(iter,:)=[t,sensors.encoders(1),sensors.encoders(2)];

    vels.lrWheelVelocitiesInMetersPerSecond=[subs(v_l, u, t),subs(v_r, u, t)]; %substitute time into u
    pause(.05); %you can add a short delay so we aren't constantly changing the velocities. 
end

vels.lrWheelVelocitiesInMetersPerSecond=[0,0];
pause(1);   
close(fig);

save("bridge_of_doom_data.mat");
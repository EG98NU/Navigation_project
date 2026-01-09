%% trajectory generation
clear
close all
clc

% timed waypoints

sf=1;

tmd = [   0.0,  9.5, 85.8, 0;
         48.0, 18.4, 85.9, 0;
        115.4, 27.2, 79.2, 0;
        179.0, 28.4, 62.9, 0;
        254.3, 35.6, 47.5, 0;
        326.7, 53.6, 33.7, 0;
        420.3, 67.5, 24.9, 0;
        480.2, 60.6,  7.7, 0 ];

tmd = [tmd(:,1), sf*tmd(:,2:4)];

% define sample time
ST = 1/100;

% creating the trajectory 
trajectory = waypointTrajectory(tmd(:,2:4), TimeOfArrival=tmd(:,1) , SampleRate=1/ST, AutoPitch=true);

% waypointInfo returns a table of specified constraints. 
tInfo = waypointInfo(trajectory);

% The trajectory object outputs the current position, velocity,
% acceleration, and angular velocity at each call. 
%% trajectory samples extraction
% Call trajectory() in a loop to get, position, velocity and 
% orientation over time. 

trajectory.reset();
%build time vector and prepare empty 
% attitude, pos, vel, acc, angvel vectors
time = 0:ST:(tInfo.TimeOfArrival(end)-ST);
time=time';
quat = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,"quaternion");
vel_n = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);
acc_n = vel_n;
pos = vel_n;
omega_n = vel_n;

count = 1;
while ~isDone(trajectory)
   [pos(count,:),quat(count),vel_n(count,:),acc_n(count,:),omega_n(count,:)] = trajectory();
   count = count + 1;
end

%% plot result
%position
figure(1);
plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2), tInfo.Waypoints(:,3),"r*")
title("Trajectory")
xlabel("North")
ylabel("East")
zlabel("Down")
grid on
axis equal
hold on
plot3(pos(:,1), pos(:,2), pos(:,3),"b" );
hold off

%% position wrt time
figure(2);
plot(time, pos(:,1), "r" );
hold on; grid on;
plot(time, pos(:,2), "g" );
plot(time, pos(:,3), "b" );
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,1),'k*');
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,2),'k*');
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,3),'k*');
title("Position")
xlabel("Time")
ylabel("[m]")
hold off
legend("North","East","Down")

%% velocity
figure(3);
plot(time, vel_n(:,1), "r" );
hold on; grid on;
plot(time, vel_n(:,2), "g" );
plot(time, vel_n(:,3), "b" );
title("Velocity in Navigation Frame")
xlabel("Time")
ylabel("[m/s]")
legend("V_n","V_e","V_d")
hold off

%% acceleration
figure(4);
plot(time, acc_n(:,1), "r" );
hold on; grid on;
plot(time, acc_n(:,2), "g" );
plot(time, acc_n(:,3), "b" );
title("Acceleration in Navigation Frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("a_n","a_e","a_d")
hold off

%% angular velocity
figure(5);
plot(time, omega_n(:,1), "r" );
hold on; grid on;
plot(time, omega_n(:,2), "g" );
plot(time, omega_n(:,3), "b" );
title("Angular Velocity in Navigation Frame")
xlabel("Time")
ylabel("[rad/s]")
legend("\omega_n","\omega_e","\omega_d")
hold off

%% Orientation (quaternion)
figure(6);
quat_c = compact(quat); %convert to array for plotting
plot(time, quat_c(:,1), "r" );
hold on; grid on;
plot(time, quat_c(:,2), "g" );
plot(time, quat_c(:,3), "b" );
plot(time, quat_c(:,4), "m" );
title("Attitude (Quaternion)")
xlabel("Time")
legend("q_0","q_1","q_2","q_3")
hold off

%% verify that acceleration and velocity are consistent with position

%integrate navigation velocity 
pos_vi = cumsum(vel_n).*ST+pos(1,:);
pos_ai = cumsum(cumsum(acc_n).*ST+vel_n(1,:)).*ST+pos(1,:);

pos_vi_t = cumtrapz(vel_n).*ST+pos(1,:);
pos_ai_t = cumtrapz(cumtrapz(acc_n).*ST+vel_n(1,:)).*ST+pos(1,:);


figure(7)
plot(time,pos(:,1),'r', ...
     time,pos(:,2),'r', ...
     time,pos(:,3),'r');
hold on;
plot(time,pos_vi(:,1),'b', ...
     time,pos_vi(:,2),'b', ...
     time,pos_vi(:,3),'b');
plot(time,pos_ai(:,1),'g', ...
     time,pos_ai(:,2),'g', ...
     time,pos_ai(:,3),'g');
plot(time,pos_vi_t(:,1),'k:', ...
     time,pos_vi_t(:,2),'k:', ...
     time,pos_vi_t(:,3),'k:');
plot(time,pos_ai_t(:,1),'m:', ...
     time,pos_ai_t(:,2),'m:', ...
     time,pos_ai_t(:,3),'m:');
hold off
title("Position Over Time")
% legend; %("North","East","Down")
xlabel("Time (seconds)")
ylabel("Position (m)")
grid on

%% smooth accelerations and recompute everything

%define a second order low pass filter 
lp_tc = 1;

lpfilt = tf(1,conv([1/lp_tc 1],[1/lp_tc 1]));
lpfilt_td = c2d(lpfilt,ST);

%filer acceleration
acc_n_f = filter(lpfilt_td.Numerator{1}, lpfilt_td.Denominator{1},acc_n);

%acceleration
figure(8)
plot(time, acc_n(:,1), "r" );
hold on; grid on;
plot(time, acc_n(:,2), "g" );
plot(time, acc_n(:,3), "b" );
plot(time, acc_n_f(:,1), "r:" , 'linewidth',2);
plot(time, acc_n_f(:,2), "g:" , 'linewidth',2);
plot(time, acc_n_f(:,3), "b:" , 'linewidth',2);
title("Acceleration in Navigation Frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("a_n","a_e","a_d","a_n filt","a_e filt","a_d filt")
hold off

%recompute velocity and position (starting from 0 velocity)
vel_n_f = cumtrapz(acc_n_f).*ST+vel_n(1,:);
pos_f = cumtrapz(vel_n_f).*ST+pos(1,:);

%plot new trajectory 
figure(9)
plot(time,pos(:,1),'r', ...
     time,pos(:,2),'r', ...
     time,pos(:,3),'r');
hold on;
plot(time,pos_f(:,1),'b:',  ...
     time,pos_f(:,2),'b:', ...
     time,pos_f(:,3),'b:', 'linewidth',2);
hold off
title("Position filtered")
legend("North","East","Down","North filtered","East filtered","Down filtered")
xlabel("Time (seconds)")
ylabel("[m]")
grid on

figure(10)
plot(time,vel_n(:,1),'r', ...
     time,vel_n(:,2),'r', ...
     time,vel_n(:,3),'r');
hold on;
plot(time,vel_n_f(:,1),'b', ...
     time,vel_n_f(:,2),'b', ...
     time,vel_n_f(:,3),'b', 'linewidth',2);
hold off
title("Velocity in Navigation Frame filtered")
legend("North","East","Down","North filtered","East filtered","Down filtered")
xlabel("Time (seconds)")
ylabel("[m/s]")
grid on

%% replace raw acc with filtered acc (and pos and vel)
use_filtered_acc = 1;
if use_filtered_acc
    acc_n = acc_n_f;
    vel_n = vel_n_f;
    pos = pos_f;
end


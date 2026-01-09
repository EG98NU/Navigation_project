 %% roll pitch yaw making
rpy_deg = zeros(3,length(out.tout));
rpy_raw_deg = zeros(3,length(out.tout));
rpy_true_deg = zeros(3,length(out.tout));

for k=1:length(out.tout)
    quat_p = quaternion(out.quat.Data(1,k),out.quat.Data(2,k),out.quat.Data(3,k),out.quat.Data(4,k));
    euler_angles = euler(quat_p,"ZYX","frame");
    % euler_angles_deg = euler_angles([3 2 1])*180/pi;
    rpy_deg(:,k) = (euler_angles([3 2 1])*180/pi)';

    quat_p_raw = quaternion(out.quat_raw.Data(1,k),out.quat_raw.Data(2,k),out.quat_raw.Data(3,k),out.quat_raw.Data(4,k));
    euler_angles_r = euler(quat_p_raw,"ZYX","frame");
    % euler_angles_r_deg = euler_angles_r([3 2 1])*180/pi;
    rpy_raw_deg(:,k) = (euler_angles_r([3 2 1])*180/pi)';

    quat_p_true = quaternion(out.quat_true.Data(k,1),out.quat_true.Data(k,2),out.quat_true.Data(k,3),out.quat_true.Data(k,4));
    euler_angles_t = euler(quat_p_true,"ZYX","frame");
    % euler_angles_t_deg = euler_angles_t([3 2 1])*180/pi;
    rpy_true_deg(:,k) = (euler_angles_t([3 2 1])*180/pi)';
end

error_rpy_raw_deg = rpy_true_deg - rpy_raw_deg;
error_rpy_deg = rpy_true_deg - rpy_deg;

MSE_rpy = [(1/length(out.tout))*vecnorm(error_rpy_deg(1,:))^2;
           (1/length(out.tout))*vecnorm(error_rpy_deg(2,:))^2;
           (1/length(out.tout))*vecnorm(error_rpy_deg(3,:))^2];

MSE_roll = ['MSE roll = ', num2str(MSE_rpy(1))];
MSE_pitch = ['MSE pitch = ', num2str(MSE_rpy(2))];
MSE_yaw = ['MSE yaw = ', num2str(MSE_rpy(3))];

close all
%% Attitude

figure(1)

subplot(3,1,1)
plot(out.tout, rpy_true_deg(1,:) ,'LineWidth',1);
hold on
plot(out.tout, rpy_raw_deg(1,:) ,'LineWidth',1);
hold on
plot(out.tout, rpy_deg(1,:) ,'LineWidth',1);
xlabel('time')
ylabel('Degrees')
legend('true roll','mech roll','estimated roll')
grid on
hold off

subplot(3,1,2)
plot(out.tout, rpy_true_deg(2,:) ,'LineWidth',1);
hold on
plot(out.tout, rpy_raw_deg(2,:) ,'LineWidth',1);
hold on
plot(out.tout, rpy_deg(2,:) ,'LineWidth',1);
xlabel('time')
ylabel('Degrees')
legend('true pitch','mech pitch','estimated pitch')
grid on
hold off

subplot(3,1,3)
plot(out.tout, rpy_true_deg(3,:) ,'LineWidth',1);
hold on
plot(out.tout, rpy_raw_deg(3,:) ,'LineWidth',1);
hold on
plot(out.tout, rpy_deg(3,:) ,'LineWidth',1);
xlabel('time')
ylabel('Degrees')
legend('true yaw','mech yaw','estimated yaw')
grid on
hold off

%% Attitude Error

figure(2)

subplot(2,1,1)
plot(out.tout, error_rpy_raw_deg ,'LineWidth',1);
title('Attitude error without EKF')
xlabel('time')
ylabel('Degrees')
legend('error roll','error pitch','error yaw')
grid on
hold on

subplot(2,1,2)
plot(out.tout, error_rpy_deg ,'LineWidth',1);
title('Attitude error with EKF')
xlabel('time')
ylabel('Degrees')
legend('error roll','error pitch','error yaw')
text(100, 0.5, MSE_roll, 'FontSize', 10, 'Color', "#0072BD");
text(200, -0.5, MSE_pitch, 'FontSize', 10, 'Color', "#D95319");
text(300, 0.5, MSE_yaw, 'FontSize', 10, 'Color', "#EDB120");
grid on
hold off

%% velocity plot definition
vel_n_raw = zeros(3,length(out.tout));
vel_n_fil = zeros(3,length(out.tout));

for k=1:length(out.tout)
    vel_n_raw(1:3,k) = out.vel_n_raw.Data(1:3,1,k);
    vel_n_fil(1:3,k) = out.vel_n.Data(1:3,1,k);
end

error_v_raw = vel_n' - vel_n_raw;
error_v_fil = vel_n' - vel_n_fil;

%% Velocity

figure(3)

subplot(3,1,1)
plot(out.tout, vel_n(:,1) ,'LineWidth',1);
hold on
plot(out.tout, vel_n_raw(1,:) ,'LineWidth',1);
hold on
plot(out.tout, vel_n_fil(1,:) ,'LineWidth',1);
xlabel('time')
ylabel('m/s')
legend('v_{North} true','v_{North} mech','v_{North} estimated')
grid on
hold off

subplot(3,1,2)
plot(out.tout, vel_n(:,2) ,'LineWidth',1);
hold on
plot(out.tout, vel_n_raw(2,:) ,'LineWidth',1);
hold on
plot(out.tout, vel_n_fil(2,:) ,'LineWidth',1);
xlabel('time')
ylabel('m/s')
legend('v_{East} true','v_{East} mech','v_{East} estimated')
grid on
hold off

subplot(3,1,3)
plot(out.tout, vel_n(:,3) ,'LineWidth',1);
hold on
plot(out.tout, vel_n_raw(3,:) ,'LineWidth',1);
hold on
plot(out.tout, vel_n_fil(3,:) ,'LineWidth',1);
xlabel('time')
ylabel('m/s')
legend('v_{Down} true','v_{Down} mech','v_{Down} estimated')
grid on
hold off

%% Velocity Error

figure(4)

subplot(2,1,1)
plot(out.tout, error_v_raw ,'LineWidth',1);
title('Velocity error without EKF')
xlabel('time')
ylabel('m/s')
legend('error v_{North}','error v_{East}','error v_{Down}')
grid on
hold on

subplot(2,1,2)
plot(out.tout, error_v_fil ,'LineWidth',1);
title('Velocity error with EKF')
xlabel('time')
ylabel('m/s')
legend('error v_{North}','error v_{East}','error v_{Down}')
grid on
hold off

%% position plot definition
pos_raw = zeros(3,length(out.tout));
pos_fil = zeros(3,length(out.tout));

for k=1:length(out.tout)
    pos_raw(1:3,k) = out.pos_raw.Data(1:3,1,k);
    pos_fil(1:3,k) = out.pos.Data(1:3,1,k);
end

error_p_raw = pos' - pos_raw;
error_p_fil = pos' - pos_fil;

%% Position

figure(5)

subplot(3,1,1)
plot(out.tout, pos(:,1) ,'LineWidth',1);
hold on
plot(out.tout, pos_raw(1,:) ,'LineWidth',1);
hold on
plot(out.tout, pos_fil(1,:) ,'LineWidth',1);
xlabel('time')
ylabel('m/s')
legend('North true','North mech','North estimated')
grid on
hold off

subplot(3,1,2)
plot(out.tout, pos(:,2) ,'LineWidth',1);
hold on
plot(out.tout, pos_raw(2,:) ,'LineWidth',1);
hold on
plot(out.tout, pos_fil(2,:) ,'LineWidth',1);
xlabel('time')
ylabel('m/s')
legend('East true','East mech','East estimated')
grid on
hold off

subplot(3,1,3)
plot(out.tout, pos(:,3) ,'LineWidth',1);
hold on
plot(out.tout, pos_raw(3,:) ,'LineWidth',1);
hold on
plot(out.tout, pos_fil(3,:) ,'LineWidth',1);
xlabel('time')
ylabel('m/s')
legend('Down true','Down mech','Down estimated')
grid on
hold off

%% Position Error

figure(6)

subplot(2,1,1)
plot(out.tout, error_p_raw ,'LineWidth',1);
title('Position error without EKF')
xlabel('time')
ylabel('m')
legend('error North','error East','error Down')
grid on
hold on

subplot(2,1,2)
plot(out.tout, error_p_fil ,'LineWidth',1);
title('Position error with EKF')
xlabel('time')
ylabel('m')
legend('error North','error East','error Down')
grid on
hold off

% %% Trajectory comparison
% figure(7);
% plot3(pos(:,1), pos(:,2), pos(:,3),"b")
% title("Trajectory")
% xlabel("North")
% ylabel("East")
% zlabel("Down")
% axis equal
% hold on
% plot3(pos_fil(1,:), pos_fil(2,:), pos_fil(3,:),"r" );
% legend('Real trajectory','Estimated trajectory')
% grid on
% hold off
% 



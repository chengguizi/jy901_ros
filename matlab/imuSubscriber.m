%%% Cheng Huimin
%%% Oct 2018

clear all;
close all;
% rosshutdown;

clc;
% rosinit('arena-glass.local');

global h_imu_ax h_imu_ay h_imu_az;
global h_imu_gx h_imu_gy h_imu_gz;
global h_mag_x h_mag_y h_mag_z;
global reset;

if (isempty(reset))
    reset = true;
end

f1 = figure();
% Configure accerlation raw plot
subplot(1,2,1);
h_imu_ax = animatedline('Color','r','MaximumNumPoints',60);
h_imu_ay = animatedline('Color','g','MaximumNumPoints',60);
h_imu_az = animatedline('Color','b','MaximumNumPoints',60);
axis tight
axis([-inf,inf,-20,20]);
title('Accelerometer Raw Data');
legend('ax','ay','az');

% Configure gyroscope raw plot
subplot(1,2,2);
h_imu_gx = animatedline('Color','r','MaximumNumPoints',60);
h_imu_gy = animatedline('Color','g','MaximumNumPoints',60);
h_imu_gz = animatedline('Color','b','MaximumNumPoints',60);
axis tight
axis([-inf,inf,-10,10]);
title('Gyroscope Raw Data');
legend('gx','gy','gz');

% Figure for attitude visualisation
% global f2;
% f2 = figure();
% axis([-10,10,-10,10,-10,10]);


% Register ROS subscriber callback
imu_sub = rossubscriber('/imu0', 'sensor_msgs/Imu' ,@imu_callback);
mag_sub = rossubscriber('/mag0', 'sensor_msgs/MagneticField', @mag_callback);

pause(5);
while true
    pause(1);
    reset = false;
    key = input('press return to reset, q and return to quit: ','s');  
    if (~isempty(key))
        break;
    end
    reset = true;
end
clear imu_sub;
clear mag_sub;
% rosshutdown;


function imu_callback(src, msg)
    persistent time_offset;
    persistent last_t last_seq;
    global h_imu_ax h_imu_ay h_imu_az;
    global h_imu_gx h_imu_gy h_imu_gz;
    global reset;
    
    time = msg.Header.Stamp.Sec + msg.Header.Stamp.Nsec / 1.0e9;
    seq = msg.Header.Seq;
    
    if isempty(time_offset)
        time_offset = time;
        last_t = time;
        last_seq = seq;
    end
    
    % relative time
    t = time - time_offset;
    
    addpoints(h_imu_ax,t,msg.LinearAcceleration.X);
    addpoints(h_imu_ay,t,msg.LinearAcceleration.Y);
    addpoints(h_imu_az,t,msg.LinearAcceleration.Z);
    
    addpoints(h_imu_gx,t,msg.AngularVelocity.X);
    addpoints(h_imu_gy,t,msg.AngularVelocity.Y);
    addpoints(h_imu_gz,t,msg.AngularVelocity.Z);

    
    if (t > last_t && seq < last_seq + 100)
        quat = [msg.Orientation.W, msg.Orientation.X, msg.Orientation.Y, msg.Orientation.Z];
        rotm = quat2rotm(quat);
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % diverge towards the sign, too small the gravity
        calib_scale = diag([0.9997600830469345,1.001241647967386,1.013021354410268]);
        calib_bias = [0 0 0]';
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        g_imu = rotm'*[0 0 9.8]';

        g_scaled_pre = [calib_scale, calib_bias]*[g_imu' 1]'
%         g_scaled = rotm*g_scaled_pre;
        
        accel = [msg.LinearAcceleration.X msg.LinearAcceleration.Y msg.LinearAcceleration.Z]';
        gyro = [msg.AngularVelocity.X msg.AngularVelocity.Y msg.AngularVelocity.Z]';
        delta_t = t - last_t;
        
        naiveIntegrate(accel, gyro, quat, t, delta_t, g_scaled_pre);
    end
    
%     if (~(seq == last_seq + 1))
%         disp(['Non-consequtive frames', num2str(seq) , ' and ' , num2str(last_seq)]);
%     end
    
%     drawnow limitrate
    
    last_t = t;
    last_seq = seq;
 end

function mag_callback(src, msg)
end
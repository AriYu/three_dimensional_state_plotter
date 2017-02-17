function three_d_state_plotter()
    clear all;
    close all;
    length = 0.5;
    % the data should be "num, x, y, z, roll, pitch, yaw".
    %state_data = csvread('~/Desktop/poseoutput_2017-02-09-22-27-30.csv');
    %state_data = csvread('~/Downloads/0214.csv');
    state_data = csvread('~/Desktop/poseoutput_2017-02-13-21-23-07.csv');
    estimated_state = kalman_estimator(state_data);
    csvwrite('./kalman_estimated.csv', estimated_state);
    % 	 axis
    tx = [length, 0.0, 0.0];
    ty = [0.0, length, 0.0];
    tz = [0.0, 0.0, length];    
    % 行数分forする
    figure(1)
    for i=1:1:size(state_data, 1)
       % generate axis vectors
       % rotate axis vectors by roll, pitch, yaw
        R = rpy2rmatrix(state_data(i, 5), state_data(i, 6), state_data(i, 7));
        tx_r = R*tx';
        ty_r = R*ty';
        tz_r = R*tz';
        origin = [state_data(i, 2), state_data(i, 3), state_data(i, 4)];
        tx_vec(1, 1:3) = origin;
        tx_vec(2, :) = tx_r + origin';
        ty_vec(1, 1:3) = origin;
        ty_vec(2, :) = ty_r + origin';
        tz_vec(1, 1:3) = origin;
        tz_vec(2, :) = tz_r + origin';
    
        hold on;
    
        p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
        set(p1,'Color','Green','LineWidth',1);
        p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
        set(p1,'Color','Blue','LineWidth',1);
        p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
        set(p1,'Color','Red','LineWidth',1);
        
    end;
    axis equal;
    title('original');
    figure(2)
    for i=1:1:size(estimated_state, 1)
       % generate axis vectors
       % rotate axis vectors by roll, pitch, yaw
        R = rpy2rmatrix(estimated_state(i, 10), estimated_state(i, 11), estimated_state(i, 12));
        tx_r = R*tx';
        ty_r = R*ty';
        tz_r = R*tz';
        origin = [estimated_state(i, 1), estimated_state(i, 2), estimated_state(i, 3)];
        tx_vec(1, 1:3) = origin;
        tx_vec(2, :) = tx_r + origin';
        ty_vec(1, 1:3) = origin;
        ty_vec(2, :) = ty_r + origin';
        tz_vec(1, 1:3) = origin;
        tz_vec(2, :) = tz_r + origin';
    
        hold on;
    
        p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
        set(p1,'Color','Green','LineWidth', 3);
        p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
        set(p1,'Color','Blue','LineWidth',3);
        p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
        set(p1,'Color','Red','LineWidth',3);
    end;
    axis equal;
    title('kalman')
    plot2d(state_data, estimated_state);
end

function R = rotx(phi)
R = [   1  0   0;
        0 cos(phi) -sin(phi);
        0 sin(phi) cos(phi)];
end

function R = roty(phi)
R = [   cos(phi)    0   sin(phi);
        0           1   0;
        -sin(phi)   0   cos(phi)];
end

function R = rotz(phi)
R = [   cos(phi)    -sin(phi)   0;
        sin(phi)    cos(phi)    0;
        0           0           1];
end

function R = rpy2rmatrix(roll, pitch, yaw)
R = rotx(roll) * roty(pitch) * rotz(yaw);
end

function estimatedM = kalman_estimator(M)
% discreat time
dt = 0.05;
% 誤差共分散行列
pEst=eye(18);
% state vector
xEst=[0 0 0 4.117 0.6762 0 0 0 0 0 0 0 0 0 0 0 0 0]';
% observation vector
z = [0 0 0 0 0 0]';
% Q small, R large : 観測値を信頼しない
% Q large, R small : 観測値を信頼する
% covariance matrix for motion
Q = diag([1000 1000 1000 1000 1000 1000 1000 1000 1000 toRadian(1) toRadian(1) toRadian(1) toRadian(1) toRadian(1) toRadian(1) toRadian(1) toRadian(1) toRadian(1)]).^2;
% covariance matrix for observation
R = diag([0.00001 0.00001 0.00001 toRadian(1) toRadian(1) toRadian(1)]).^2;
% state transition matrix
A = [1 0 0 dt 0  0  0.5*(dt)^2 0          0
     0 1 0 0  dt 0  0          0.5*(dt)^2 0
     0 0 1 0  0  dt 0          0          0.5*(dt)^2
     0 0 0 1  0  0  dt         0          0
     0 0 0 0  1  0  0          dt         0
     0 0 0 0  0  1  0          0          dt
     0 0 0 0  0  0  1          0          0
     0 0 0 0  0  0  0          1          0
     0 0 0 0  0  0  0          0          1];
A2 = blkdiag(A, A);
% observation matrix
H = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
     0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0
     0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0];
 estimatedM = xEst';
 for i=2:size(M, 1)
    % time update
    xPred = A2*xEst; % project the state ahead
    pPred = A2*pEst*A2.' + Q; % project the covariane ahead
    % measurement update
    z = [M(i, 2) M(i, 3) M(i, 4) M(i, 5) M(i, 6) M(i, 7)]';
    K = pPred*H.'*inv(H*pPred*H.' + R); % compute the kalman gain
    xEst = xPred + K*(z-H*xPred); % Update state estimate
    pEst = (eye(18) - K*H)*pPred; % Update the covariance
    estimatedM = vertcat(estimatedM, xEst'); %#ok<*AGROW>
 end
 size(estimatedM)
end

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;
end

function plot2d(data, data2)
 % 2次元で各変数を比較
    % x
    figure(3)
    plot(data(:,1), data(:,2), data(:,1), data2(:,1))
    title('x')
    % y
    figure(4)
    plot(data(:,1), data(:,3), data(:,1), data2(:,2))
    title('y')
    % z
    figure(5)
    plot(data(:,1), data(:,4), data(:,1), data2(:,3))
    title('z')
    % roll
    figure(6)
    plot(data(:,1), data(:,5), data(:,1), data2(:,10))
    title('roll')
    % pitch
    figure(7)
    plot(data(:,1), data(:,6), data(:,1), data2(:,11))
    title('pitch')
    % yaw
    figure(8)
    plot(data(:,1), data(:,7), data(:,1), data2(:,12))
    title('yaw')
    % v
    figure(9)
    a = 10;
    b = ones(1, a);
    diff_x = diff(data(:,2));
    diff_y = diff(data(:,3));
    diff_x(1)/0.05
    diff_y(1)/0.05
    movemean_x = filter(b, a, diff_x);
    movemean_y = filter(b, a, diff_y);
    p1 = plot(data(:,1)*0.05/60.0,  sqrt(data2(:,4).^2.0 + data2(:,5).^2)*(18/5), (1:length(diff_x))*0.05/60.0, sqrt((diff_x/0.05).^2.0 + (diff_y/0.05).^2.0)*18/5, (1:length(movemean_x))*0.05/60.0, sqrt((movemean_x/0.05).^2.0 + (movemean_y/0.05).^2.0)*18/5)
    set(p1,'LineWidth',1);
    legend('カルマンフィルタ','差分', '移動平均','Location','southeast')
    title('車速')
    axis([-inf,inf,0,80])
    xlabel('time [minutes]')
    ylabel('velocity [km/h]')
end


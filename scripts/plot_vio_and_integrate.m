close all;
clear all;
clc;


%dataset = '/home/pang/maplab_linux/calibrate_result/2018-10-19_07-33-26/';
dataset = '/home/pang/maplab_linux/calibrate_result/2018-10-26_10-59-27//';
vio_odom = load([dataset '/vio_odom.txt']);
original_integrate_odom = load([dataset '/original_integrate_odom.txt']);
re_integrate_odom = load([dataset '/re_integrate_odom.txt']);

data1 = vio_odom;
data2 = original_integrate_odom;
data3 = re_integrate_odom;

%%
figure(1);

plot3(data1(:,1), data1(:,2), data1(:,3),'r');
hold on;
plot3(data2(:,1), data2(:,2), data2(:,3),'g');
hold on;
plot3(data3(:,1), data3(:,2), data2(:,3),'b');
axis equal;
title('trajecotries');
xlabel('x');
ylabel('y');
legend('vio','uncalib','calib');
view(0,90);

%%
figure(2);
t  =1: size(data1);
subplot(3,1,1);
plot(t, data1(:,1),'r', t, data2(:,1),'g',t, data3(:,1),'b');
xlabel('x');

subplot(3,1,2);
plot(t, data1(:,2),'r', t, data2(:,2),'g',t, data3(:,2),'b');
xlabel('y');
subplot(3,1,3);
plot(t, data1(:,3),'r', t, data2(:,3),'g', t, data3(:,3),'b');
xlabel('theta');

%%

diff_vio_pose = diff(data1);
for i = 1: size(diff_vio_pose,1)
    if diff_vio_pose(i,3) > pi
        diff_vio_pose(i,3) = diff_vio_pose(i,3) - 2*pi;
    elseif diff_vio_pose(i,3) < -pi
        diff_vio_pose(i,3) = diff_vio_pose(i,3) + 2*pi;
    end
end


diff_original_odom_pose = diff(data2);
for i = 1: size(diff_original_odom_pose,1)
    if diff_original_odom_pose(i,3) > pi
        diff_original_odom_pose(i,3) = diff_original_odom_pose(i,3) - 2*pi;
    elseif diff_original_odom_pose(i,3) < -pi
        diff_original_odom_pose(i,3) = diff_original_odom_pose(i,3) + 2*pi;
    end
end

diff_re_odom_pose = diff(data3);
for i = 1: size(diff_re_odom_pose,1)
    if diff_re_odom_pose(i,3) > pi
        diff_re_odom_pose(i,3) = diff_re_odom_pose(i,3) - 2*pi;
    elseif diff_re_odom_pose(i,3) < -pi
        diff_re_odom_pose(i,3) = diff_re_odom_pose(i,3) + 2*pi;
    end
end

diff_t = 1: size(diff_vio_pose,1);

figure(3);
t  =1: size(diff_vio_pose);
subplot(3,1,1);
plot(t, diff_vio_pose(:,1),'r', t, diff_original_odom_pose(:,1),'g', t, diff_re_odom_pose(:,1),'b' );

subplot(3,1,2);
plot(t, diff_vio_pose(:,2),'r', t, diff_original_odom_pose(:,2),'g', t, diff_re_odom_pose(:,2),'b');

subplot(3,1,3);
plot(t, diff_vio_pose(:,3),'r', t, diff_original_odom_pose(:,3),'g', t, diff_re_odom_pose(:,3),'b');

% 
% %%
% [r,lags] = xcorr(diff_original_odom_pose(:,3),diff_vio_pose(:,3));
% figure(4)
% plot(lags,r)
% [~,I] = max(abs(r));
% lagDiff = lags(I)
% 

%%
figure(5);

subplot(2,1,1);
encoder_data = data2(:,4:5);
diff_encoder_data = diff(encoder_data);
plot(t, diff_encoder_data(:,1),'r',t, diff_encoder_data(:,2),'g');
subplot(2,1,2);
plot(t, diff_encoder_data(:,1)-diff_encoder_data(:,2));



%plot_simulate

close all;
clear all;
clc;


encoder_integrate = load('/home/pang/encoder_integrate_odom.txt');
simulated_odom = load('/home/pang/simulated_odom.txt');

figure(1);

data1 = encoder_integrate;
data2 = simulated_odom;
plot3(data1(:,1), data1(:,2), data1(:,3),'r');
hold on;
plot3(data2(:,1), data2(:,2), data2(:,3),'g');
axis equal;
xlabel('x');
ylabel('y');


%%
figure(2);
t  =1: size(data1);
subplot(3,1,1);
plot(t, data1(:,1),'r', t, data2(:,1),'g');

subplot(3,1,2);
plot(t, data1(:,2),'r', t, data2(:,2),'g');

subplot(3,1,3);
plot(t, data1(:,3),'r', t, data2(:,3),'g');

%%
diff_odom_pose = diff(data1);
for i = 1: size(diff_odom_pose,1)
    if diff_odom_pose(i,3) > pi
        diff_odom_pose(i,3) = diff_odom_pose(i,3) - 2*pi;
    elseif diff_odom_pose(i,3) < -pi
        diff_odom_pose(i,3) = diff_odom_pose(i,3) + 2*pi;
    end
end



diff_vio_pose = diff(data2);

for i = 1: size(diff_vio_pose,1)
    if diff_vio_pose(i,3) > pi
        diff_vio_pose(i,3) = diff_vio_pose(i,3) - 2*pi;
    elseif diff_vio_pose(i,3) < -pi
        diff_vio_pose(i,3) = diff_vio_pose(i,3) + 2*pi;
    end
end



diff_t = 1: size(diff_vio_pose,1);

figure(3);
t  =1: size(diff_vio_pose);
subplot(3,1,1);
plot(t, diff_odom_pose(:,1),'r', t, diff_vio_pose(:,1),'g');

subplot(3,1,2);
plot(t, diff_odom_pose(:,2),'r', t, diff_vio_pose(:,2),'g');

subplot(3,1,3);
plot(t, diff_odom_pose(:,3),'r', t, diff_vio_pose(:,3),'g');


%%
[r,lags] = xcorr(diff_odom_pose(:,3),diff_vio_pose(:,3));
figure()
plot(lags,acor)
a3 = gca;



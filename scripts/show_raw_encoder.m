close all;
clear all;
clc;

% show_raw_encoder
%dataset_path = '/home/pang/dataset/ninebot/evt_3_3/calibrate_8';
%dataset_path = '/home/pang/dataset/ninebot/evt_2_1/calibrate_1';
%dataset_path = '/home/pang/dataset/ninebot/evt_3_7/calibrate_1';
%dataset_path = '/home/pang/dataset/ninebot/test_3_7';
%dataset_path = '/home/pang/dataset/ninebot/evt_3_7/2018-10-24_10-26-54__calibrate';
dataset_path = '/home/pang/dataset/ninebot/evt_3_7/2018-10-26_10-44-43__calibrate';
% fisheye_ts_file = [dataset_path '/fisheye_timestamps.txt'];
% fisheye_ts = load(fisheye_ts_file);
% fisheye_ts_diff = diff(fisheye_ts(:,2));
odom_file = [dataset_path '/odom.txt'];
odom = load(odom_file);
ts = odom(:,1);
dt = diff(ts);
t = 1: size(dt, 1);
figure(1);
plot(t, dt);

odom_data = odom(:,7:8);
diff_odom = diff(odom_data);
figure(2);
plot(t, diff_odom(:,1),'r', t, diff_odom(:,2), 'g');

% remove duplicated
none_zero_dt_indice = find(dt>0);
dt = dt(none_zero_dt_indice);
velocity_left = diff_odom(none_zero_dt_indice,1)./dt;
velocity_right = diff_odom(none_zero_dt_indice,2)./dt;
figure(3);
tt = 1: size(dt, 1);
plot(tt,velocity_left,'r', tt, velocity_right,'g');



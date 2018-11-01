% check_encoder_raw_readings
clear all;
close all;
clc;

% 
dataset_encoder_file = '/home/pang/dataset/ninebot/evt_3_2/calibrate_1/odom.txt';
odom_flow_record_file_pc = '/home/pang/maplab_ws/src/maplab/algorithms/odom_extrinsic_calibrate/calibrate_dataset/evt_3_2/calibrate_1/encoder_meas.txt';
odom_flow_record_file_robot = '/home/pang/dataset/ninebot/evt_3_2/calibrate_on_robot/calibrate_1/encoder_meas.txt';
odom_solver_record_file_pc = '/home/pang/maplab_ws/src/maplab/algorithms/odom_extrinsic_calibrate/calibrate_dataset/evt_3_2/calibrate_1/original_integrate_odom.txt';
odom_solver_record_file_robot = '/home/pang/dataset/ninebot/evt_3_2/calibrate_on_robot/calibrate_1/original_integrate_odom.txt';

dataset_encoder = load(dataset_encoder_file);
dataset_encoder = dataset_encoder(:, 7:8);

odom_flow_record_pc = load(odom_flow_record_file_pc);
odom_flow_record_pc = odom_flow_record_pc(:,2:3);

odom_flow_record_robot = load(odom_flow_record_file_robot);
odom_flow_record_robot = odom_flow_record_robot(:,2:3);

odom_solver_pc = load(odom_solver_record_file_pc);
odom_solver_pc = odom_solver_pc(:,4:5);

odom_solver_robot = load(odom_solver_record_file_robot);
odom_solver_robot = odom_solver_robot(:,4:5);


figure(1);
diff1 = diff(dataset_encoder);
t1 = 1: size(diff1,1);
plot(t1, diff1(:,1),'r',t1, diff1(:,2),'b');
title('dataset');

figure(2);
diff2 = diff(odom_flow_record_pc);
t2 = 1: size(diff2,1);
plot(t2, diff2(:,1),'r',t2, diff2(:,2),'b');
title('PC odom-flow');

figure(3);
diff3 = diff(odom_flow_record_robot);
t3 = 1: size(diff3,1);
plot(t3, diff3(:,1),'r',t3, diff3(:,2),'b');
title('Robot odom-flow');

figure(4);
diff4 = diff(odom_solver_pc);
t4 = 1: size(diff4,1);
plot(t4, diff4(:,1),'r',t4, diff4(:,2),'b');
title('PC odom-solver');

figure(5);
diff5 = diff(odom_solver_robot);
t5 = 1: size(diff5,1);
plot(t5, diff5(:,1),'r',t5, diff5(:,2),'b');
title('Robot odom-solver');
clear; clc; close all; 

filename = 'data_gyro.csv';

T = readtable(filename);

Arr = table2array(T) ;
s = size(Arr(:,1))
t = Arr(1000:s(1),1);
dot_roll = Arr(1000:s(1),2);
dot_pitch = Arr(1000:s(1),3);
dot_yaw = Arr(1000:s(1),4);


subplot(2,2,1);
hold on
plot(t,dot_roll, 'k.');
subplot(2,2,2);
hold on
plot(t,dot_pitch, 'k.');
subplot(2,2,3);
hold on
plot(t,dot_yaw, 'k.');

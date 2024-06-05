clc; clear;
%% Trunk
H = 0.083;
L = 0.66;
W = 0.25;

m = 28;

Ixx = m*(W^2+H^2)/12;
Iyy = m*(L^2+H^2)/12;
Izz = m*(L^2+W^2)/12;

display(Ixx);
display(Iyy);
display(Izz);

%% Hip
H = 0.083;
L = 0.197;
W = 0.11;

m = 2.5;

Ixx = m*(W^2+H^2)/12;
Iyy = m*(L^2+H^2)/12;
Izz = m*(L^2+W^2)/12;

display(Ixx);
display(Iyy);
display(Izz);

%% Thigh
H = 0.07;
L = 0.25;
W = 0.02;

m = 1.017;

Ixx = m*(W^2+H^2)/12;
Iyy = m*(L^2+H^2)/12;
Izz = m*(L^2+W^2)/12;

display(Ixx);
display(Iyy);
display(Izz);

%% Shank
H = 0.07;
L = 0.25;
W = 0.02;

m = 0.143;

Ixx = m*(W^2+H^2)/12;
Iyy = m*(L^2+H^2)/12;
Izz = m*(L^2+W^2)/12;

display(Ixx);
display(Iyy);
display(Izz);
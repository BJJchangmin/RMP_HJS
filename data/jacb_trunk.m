clc;
syms a b W
% a = 0.34;
% b= 0.32;
% W = 0.25;

Jp = [0.25 0.25 0.25 0.25;
      1 / W -1 / W 1 / W -1 / W;
      -1 / (2 * b) -1 / (2 * b) 1 / (2 * a) 1 / (2 * a);
      1 -1 -1 1];

Jp_inv = inv(Jp);

disp(Jp_inv)

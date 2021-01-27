clear all
close all
clc

s=tf('s');

H_engine = (1)/(s + 1);
H_integrator = 1/s;

SS = load('f16long5000ft300fts.mat', 'SS_long_lo');
SS_long_lo = SS.SS_long_lo;
A = SS_long_lo.A(1:5,1:5);
B = SS_long_lo.A(1:5,6:7);
C = SS_long_lo.C(:,1:5);
D = SS_long_lo.D;

B_speed = B(:,1);
C_speed = C(3,:);
D_speed = D(3,1);

P = 100;
I = 10;
D = 20;
C = P + I/s + D*s;


ss_speed = ss(A, B_speed, C_speed, D_speed);
H_speed = minreal(tf(ss_speed));
H_1 = H_engine*H_speed;

% rltool(H_1)

H_1_cl = minreal(C*H_1/(1+C*H_1));

figure
impulse(H_1_cl, 20)





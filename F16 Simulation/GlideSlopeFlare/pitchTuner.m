clear all
close all
clc

s=tf('s');

H_elev = (20.2)/(s + 20.2);
H_integrator = 1/s;

SS = load('f16long5000ft300fts.mat', 'SS_long_lo');
SS_long_lo = SS.SS_long_lo;
A = SS_long_lo.A(1:5,1:5);
B = SS_long_lo.A(1:5,6:7);
C = SS_long_lo.C(:,1:5);
D = SS_long_lo.D;

B_pitch_angle = B(:,2);
C_pitch_angle = C(2,:);
D_pitch_angle = D(2,2);


B_pitch_rate = B(:,2);
C_pitch_rate = C(5,:);
D_pitch_rate = D(5,2);

H_one = 1;
ss_pitch_angle = ss(A, B_pitch_angle, C_pitch_angle, D_pitch_angle);
H_pitch_angle = minreal(tf(ss_pitch_angle));
H_1_pitch_angle = -H_elev*H_pitch_angle;

ss_pitch_rate = ss(A, B_pitch_rate, C_pitch_rate, D_pitch_rate);
H_pitch_rate = minreal(tf(ss_pitch_rate));
H_1 = H_elev*H_pitch_rate;

K_1 = 1.3;
K_2 = -0.74;
H_1_cl = minreal(H_integrator*K_2*H_1/(1+K_2*H_1));
H_2_cl = minreal(K_1*H_1_cl/(1+K_1*H_1_cl));

rltool()

figure
step(H_2_cl, 90);







% H_1 = H_elev*H_pitch_rate;
% rltool(H_1);
% 
% H_1_cl = minreal(K_1*H_1/(1+K_1*H_1));
% 
% figure
% step(H_1_cl, 30)
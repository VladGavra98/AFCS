% Trim conditions:
h = 10000 *0.3048;
V = 900 * 0.3048;
g =  9.80665;

s = tf('s');
dt = 0.001;
 
%% Longitudinal system:
%From task 5, we get the initial longitudinal system

A_long_initial= [0.000e+00    9.000e+02    0.000e+00   -9.000e+02    0.000e+00    0.000e+00    0.000e+00; 
                0.000e+00    0.000e+00    0.000e+00    0.000e+00    1.000e+00    0.000e+00    0.000e+00; 
                2.420e-04   -3.217e+01   -1.718e-02    6.397e+01    5.824e-01    1.570e-03    5.250e-01; 
                1.118e-06   -1.748e-13   -7.940e-05   -1.349e+00    9.322e-01   -3.239e-09   -2.833e-03; 
                2.235e-20    0.000e+00   -1.586e-18   -5.916e+00   -1.819e+00    0.000e+00   -4.351e-01; 
                0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00   -1.000e+00    0.000e+00; 
                0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00   -2.020e+01];
      
 
A_long_initial_corrected= [A_long_initial(2, 2:end);
                           A_long_initial(3, 2:end);
                           A_long_initial(4, 2:end);
                           A_long_initial(5, 2:end);
                           A_long_initial(6, 2:end);
                           A_long_initial(7, 2:end)];
                                              
B_long_initial=   [0.000e+00    0.000e+00; 
                   0.000e+00    0.000e+00; 
                   0.000e+00    0.000e+00; 
                   0.000e+00    0.000e+00; 
                   0.000e+00    0.000e+00; 
                   1.000e+00    0.000e+00; 
                   0.000e+00    2.020e+01];
               
C_long_initial = [1.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    1.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00];                

D_long_initial= [0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00];              

%-------------- Eliminating elevator actuator d_e and engine dynamics d_t
%from A and B and keep only u_el as input -------------------------------

A_long_a_c= [A_long_initial_corrected(1,1:4);
             A_long_initial_corrected(2,1:4);
             A_long_initial_corrected(3,1:4);
             A_long_initial_corrected(4,1:4)];
         
B_long_a_c= [A_long_initial_corrected(1:4,6)];

A_complete= [[A_long_a_c; [0 0 0 0]],[B_long_a_c; -20.2]];  

C_long_a_c = [C_long_initial(2:end,2), C_long_initial(2:end,3),C_long_initial(2:end,4),C_long_initial(2:end,5)]; %corrected for altitude
 
D_long_a_c= D_long_initial(2:end,1);
 
 %% Creating longitudinal state-space
 sys = ss(A_long_a_c,B_long_a_c,C_long_a_c,D_long_a_c);
 
 sys.InputName   = {'delta_e'};
 sys.OutputName = {'theta','V','alpha','q'};
 sys.StateName  = {'theta','V','alpha','q'};
 
 %Extracting the poles of the system
 poles     = pole(sys);
 
 % Determining the motion characteristics of the phugoid and short period
 [wn,zeta] = damp(sys);
 
 
 wn_phugoid = wn(1);
 zeta_phugoid = zeta(1);
 T_1_2_phugoid = log(0.5)/real(poles(1));
 P_phugoid = 2*pi/imag(poles(1));
 
 
 wn_short_period= wn(3);
 zeta_short_period= zeta(3);
 T_1_2_short_period = log(0.5)/real(poles(3));
 P_short_period = 2*pi/imag(poles(3));
 
 %%       Short Period Reduced Model
 
A_sp  = [A_long_a_c(3:end,3) , A_long_a_c(3:end,4)];
B_sp  = [B_long_a_c(3:end)];
C_sp  = [C_long_a_c(3:end,3) , C_long_a_c(3:end,4)];
D_sp  = [0;0];


sys_sp  = ss(A_sp,B_sp,C_sp,D_sp);

sys_sp.InputName   = {'delta_e'};
sys_sp.OutputName  = {'alpha','q'};
sys_sp.StateName   = {'alpha','q'};

H_q_de_op       = minreal(tf(sys_sp('q')));
H_alpha_de_op   = minreal(tf(sys_sp('alpha')));

[wn_sp,zeta_sp] = damp(sys_sp);
wn_sp_init    = wn_sp(1);
zeta_sp_init  = zeta_sp( 1);
aux = cell2mat(H_q_de_op.num);
T_theta2 = aux(2)/aux(3);
T_theta2_init = T_theta2;



%% Requirements:
wn_sp_r    = 0.03 * V;
T_theta2_r = 1/(0.75*wn_sp_r);
zeta_sp_r  = 0.5;


poles_r = [complex(-zeta_sp_r * wn_sp_r ,- wn_sp_r * sqrt(1- zeta_sp_r^2))    complex(-zeta_sp_r * wn_sp_r ,+ wn_sp_r * sqrt(1- zeta_sp_r^2))];

%% Calculate gains:
K = place(A_sp,B_sp, poles_r);
  

K_alpha = K(1);
K_q     = K(2);

%% Build closed-loop system:
sys_sp_cl = ss(A_sp - B_sp*K, B_sp, C_sp, D_sp);

% disp("Poles of closed system: ")
poles_cl = pole(sys_sp_cl);

sys_sp_cl.InputName   = {'delta_e'};
sys_sp_cl.OutputName  = {'alpha','q'};
sys_sp_cl.StateName   = {'alpha','q'};
 

H_q_de_cl       = minreal(tf(sys_sp_cl('q')));
H_alpha_de_cl  = minreal(tf(sys_sp_cl('alpha')));

%% Extract pole values:
[wn_sp,zeta_sp] = damp(sys_sp_cl);
wn_sp    = wn_sp(1);
zeta_sp  = zeta_sp(1);

%% Lead-lag filter:

tau_d  = T_theta2_r;
tau_i  = T_theta2;
H_ll = (tau_d*s +1)/(1+tau_i*s);

H_q_de = minreal(H_ll * H_q_de_cl);
aux = cell2mat(H_q_de.num);
T_theta2 = aux(2)/aux(3);



%% Gust Requirement (MIL-F-8785C):
gust = 4.572;
disturbance = abs(gust / V);   % bad gust 

dist_alpha = K_alpha * atan(disturbance);
dist_q     = K_q * atan(disturbance);



%% Control Anticipation Parameter:
CAP_init = wn_sp_init^2/((V/g)*(1/T_theta2)); %initial value of the CAP, before stability augmentation

CAP = wn_sp^2/((V/g)*(1/T_theta2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 %% ------------------------- Plotting ------------------------------

 %% CAP and short period damping ratio requirements

%% Flight Phase A

Flight_phase_A_level_1 = [0.35,0.28,1.3-0.35,3.6-0.28]; %x,y,width, height
Flight_phase_A_level_2 = [0.25,0.16,2-0.25,10-0.16];
Flight_phase_A_level_3 = [0.15,0.01,10-0.15,10-0.01];


figure(1)

subplot(2,2,1);
grid on;
r_1=rectangle('Position',Flight_phase_A_level_3,'FaceColor',[39/255, 64/255, 139/255, 0.5]); hold on;
r_2=rectangle('Position',Flight_phase_A_level_2,'FaceColor',[79/255, 148/255, 205/255, 0.6]); hold on;
r_3=rectangle('Position',Flight_phase_A_level_1,'FaceColor',[0, 1, 1, 0.5]); hold on;
scatter(zeta_sp,CAP, 'MarkerEdgeColor',[0 .3 .3],...
              'MarkerFaceColor',[0 204/255 0])
scatter(zeta_sp_init,CAP_init,'r', "filled")
set(gca, 'XScale','log')
set(gca, 'YScale','log')
xlim([0.1 10])
ylim([0.01 10])
ylabel('CAP [1/(g \cdot s^{2})]', 'FontSize',13);
xlabel('\zeta_{sp} [-]', 'FontSize',13);
title('Flight Phase Category A', 'FontSize',14)

%% Flight Phase B
Flight_phase_B_level_1 = [0.3,0.085,2-0.3,3.6-0.085]; %x,y,width, height
Flight_phase_B_level_2 = [0.2,0.038,2-0.2,10-0.038];
Flight_phase_B_level_3 = [0.15,0.01,10-0.15,10-0.01];

subplot(2,2,2);
grid on;
r_1=rectangle('Position',Flight_phase_B_level_3,'FaceColor',[39/255, 64/255, 139/255, 0.5]); hold on;
r_2=rectangle('Position',Flight_phase_B_level_2,'FaceColor',[79/255, 148/255, 205/255, 0.6]); hold on;
r_3=rectangle('Position',Flight_phase_B_level_1,'FaceColor',[0, 1, 1, 0.5]); hold on;
scatter(zeta_sp,CAP, 'MarkerEdgeColor',[0 .3 .3],...
              'MarkerFaceColor',[0 204/255 0])
scatter(zeta_sp_init,CAP_init,'r', "filled")
set(gca, 'XScale','log')
set(gca, 'YScale','log')
xlim([0.1 10])
ylim([0.01 10])
ylabel('CAP [1/(g \cdot s^{2})]', 'FontSize',13);
xlabel('\zeta_{sp} [-]', 'FontSize',13);
title('Flight Phase Category B', 'FontSize',14)
    
%% Flight Phase C

Flight_phase_C_level_1 = [0.35,0.16,1.3-0.35,3.6-0.16]; %x,y,width, height
Flight_phase_C_level_2 = [0.25,0.05,2-0.25,10-0.05];
Flight_phase_C_level_3 = [0.15,0.01,10-0.15,10-0.01];


subplot(2,2,3);

grid on;
r_1=rectangle('Position',Flight_phase_C_level_3,'FaceColor',[39/255, 64/255, 139/255, 0.5]); hold on;
r_2=rectangle('Position',Flight_phase_C_level_2,'FaceColor',[79/255, 148/255, 205/255, 0.6]); hold on;
r_3=rectangle('Position',Flight_phase_C_level_1,'FaceColor',[0, 1, 1, 0.5]); hold on;
scatter(zeta_sp,CAP, 'MarkerEdgeColor',[0 .3 .3],...
              'MarkerFaceColor',[0 204/255 0])
scatter(zeta_sp_init,CAP_init,'r', "filled")
set(gca, 'XScale','log')
set(gca, 'YScale','log')
xlim([0.1 10])
ylim([0.01 10])
ylabel('CAP [1/(g \cdot s^{2})]', 'FontSize',13);
xlabel('\zeta_{sp} [-]', 'FontSize',13);
title('Flight Phase Category C', 'FontSize',14)

%% Comparing the short period pitch rate time response of the 4 state mdel and the 2 state model
dt=0.001;

t_short_period = 0:dt:10;
y_short_period_4 = step(sys, t_short_period);
y_short_period_2 = step(sys_sp, t_short_period);

figure(2)
     f=4;
     plot(t_short_period, y_short_period_4(:,4), 'b-', 'LineWidth',1.5);
     hold on;
     plot(t_short_period, y_short_period_2(:,2), '--r', 'LineWidth',1.5);
     grid on;
     hold on;
     legend("4 state model", "reduced 2 state model");
     title("Pitch Rate - q [ \circ/s]");
     xlabel("Time [s]")
     ylabel("Pitch Rate - q [ \circ/s]");
     
     
 


%%  pitch rate time response to step input graph Fig 6.2 a)
figure(3)
t = 0:0.01:20;
u = heaviside(t)-heaviside(t-10);
[y_values_init, x_values_init]  = lsim(-1*H_q_de_cl,u,t);
[y_values, x_values]  = lsim(-1*H_q_de,u,t);
plot(t,u, "k-", "LineWidth", 1);
hold on;
grid on;
plot(x_values_init,y_values_init,"r-", "LineWidth", 1.5);
plot(x_values,y_values, "b-", "LineWidth", 1.5);
legend("Input", "Initial pitch rate time response", "Current pitch rate time response")
axis([0 20 -1.5 2]);
title("Pitch Rate - q [ \circ/s]");
xlabel("Time [s]")
ylabel("Pitch Rate - q [ \circ/s]");


%% Pitch angle time response to step input graph Fig 6.2 b)
figure(4)
t = 0:0.01:20;
u = heaviside(t)-heaviside(t-10);
[y_values_a_init, x_values_a_init] = lsim(-1*H_q_de_cl*(1/s),u,t);
[y_values_a, x_values_a] = lsim(-1*H_q_de*(1/s),u,t);
hold on;
grid on;
plot(x_values_a_init, y_values_a_init, "r-", "LineWidth", 1.5);
plot(x_values_a, y_values_a, "b-", "LineWidth", 1.5);
plot([0,10,20],[0,y_values_a(2000),y_values_a(2000)],  "k-", "LineWidth", 1)
legend("Initial pitch angle time response", "Current pitch angle time response", "Steady state pitch attitude")
title("Pitch Angle - \theta [\circ]");
xlabel("Time [s]")
ylabel("Pitch Angle - \theta [\circ]");


%% Dropback Criterion
DB_q_ss_init = T_theta2_init - (2*zeta_sp_init)/(wn_sp_init);
DB_q_ss = T_theta2 - (2*zeta_sp)/(wn_sp);

q_m_init = max(y_values_init); %maximum pitch rate
q_m = max(y_values); %maximum pitch rate


index = find(x_values==9.99);
q_s = y_values(index); %steady state value of pitch rate


q_m_q_s_ratio_init = q_m_init/ q_s;
q_m_q_s_ratio = q_m/ q_s;

%% Criterion for the tracking task (DB graph) Fig 6.3

figure(5)
A = [0 0] ;
B = [0  3] ;
C = [0.06 3] ;
D = [0.3 0] ;  
coords = [A ; B; C; D]; 
xlim([-0.4, 0.6]);
ylim([1, 4]);
patch(coords(:,1), coords(:,2),'c', 'FaceAlpha',0.6);
hold on;
grid on;
scatter(DB_q_ss_init,q_m_q_s_ratio_init,"filled",'r');
scatter(DB_q_ss,q_m_q_s_ratio,'MarkerEdgeColor',[0 .3 .3],...
              'MarkerFaceColor',[0 204/255 0]');
hold on;
txt = ['Satisfactory'];
text(0.03,1.2,txt)
txt = ['Sluggish'];
text(-0.25,2.5,txt)
txt = ['Continuous bobbling'];
text(0.3,3.5,txt)
txt = ['Abrupt bobble tendency'];
text(0.3,2.25,txt)
xline(0, "k");
title("Criterion for the tracking task");
xlabel('DB/q_s [s]')
ylabel('q_m/q_s [-]')
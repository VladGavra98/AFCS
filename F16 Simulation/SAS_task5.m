% Trim conditions:
h = 10000 *0.3048;
V = 900 * 0.3048;
g =  9.80665;

s = tf('s');
dt = 0.001;
 
%% Longitudinal system:
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
               
% same for B, we need to take out the first row which correspnds to the height 
% and put rearrange the matrix; futhermore, y=[u_eng u_el]^T but we need it
% to be y=[u_el u_eng], so we switch the clumns of B


% B_long_initial_corrected= [B_long_initial(3,:); 
%                            B_long_initial(4,:);
%                            B_long_initial(2,:);
%                            B_long_initial(5,:);
%                            B_long_initial(7,:);
%                            B_long_initial(6,:)];
                       
% B_long_initial_corrected= [B_long_initial(2:end,:)];
% 
% B_long_initial_corrected= [B_long_initial_corrected(:,2), B_long_initial_corrected(:,1)];                        

%-------------- Eliminating elevator actuator d_e and engine dynamics d_t
%from A and B and keep only u_el as input -------------------------------

A_long_a_c= [A_long_initial_corrected(1,1:4);
             A_long_initial_corrected(2,1:4);
             A_long_initial_corrected(3,1:4);
             A_long_initial_corrected(4,1:4)];
         
B_long_a_c= [A_long_initial_corrected(1:4,6)];

A_complete= [[A_long_a_c; [0 0 0 0]],[B_long_a_c; -20.2]];  


C_long_initial = [1.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    1.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00]; 

C_long_a_c = [C_long_initial(2:end,2), C_long_initial(2:end,3),C_long_initial(2:end,4),C_long_initial(2:end,5)]; %corrected for altitude


D_long_initial= [0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00]; 
         
 D_long_a_c= D_long_initial(2:end,1);
 
 % Create longitudinal state-space
 sys = ss(A_long_a_c,B_long_a_c,C_long_a_c,D_long_a_c);
 
 sys.InputName   = {'delta_e'};
 sys.OutputName = {'theta','V','alpha','q'};
 sys.StateName  = {'theta','V','alpha','q'};
 
 %Extracting the poles of the system
 poles     = pole(sys)
 
 % Determining the motion characteristics of the phugoid and short period
 [wn,zeta] = damp(sys)
 
 
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
wn_sp    = wn_sp(1);
zeta_sp  = zeta_sp( 1);
aux = cell2mat(H_q_de_op.num);
T_theta2 = aux(2)/aux(3)



%% Requirements:
wn_sp_r    = 0.03 * V;
T_theta2_r = 1/(0.75*wn_sp_r);
zeta_sp_r  = 0.5;


poles_r = [complex(-zeta_sp_r * wn_sp_r ,- wn_sp_r * sqrt(1- zeta_sp_r^2))    complex(-zeta_sp_r * wn_sp_r ,+ wn_sp_r * sqrt(1- zeta_sp_r^2))];

%% Calculate gains:
K = place(A_sp,B_sp, poles_r);
  

K_alpha = K(1)
K_q     = K(2)

%% Build closed-loop system:
sys_sp_cl = ss(A_sp - B_sp*K, B_sp, C_sp, D_sp);

disp("Poles of closed system: ")
poles_cl = pole(sys_sp_cl)

sys_sp_cl.InputName   = {'delta_e'};
sys_sp_cl.OutputName  = {'alpha','q'};
sys_sp_cl.StateName   = {'alpha','q'};
 

H_q_de_cl       = minreal(tf(sys_sp_cl('q')));
H_alpha_de_cl  = minreal(tf(sys_sp_cl('alpha')));

%% Extract pole values:
[wn_sp,zeta_sp] = damp(sys_sp_cl);
wn_sp    = wn_sp(1)
zeta_sp  = zeta_sp( 1)

%% Wash-out filter:

tau_d  = T_theta2_r;
tau_i  = T_theta2;
H_ll = (tau_d*s +1)/(1+tau_i*s);

H_q_de = minreal(H_ll * H_q_de_cl);
aux = cell2mat(H_q_de.num);

T_theta2 = aux(2)/aux(3)


%% Gust Requirement (MIL-F-8785C):
gust = 4.572;
disturbance = abs(gust / V);   % bad gust 

dist_alpha = K_alpha * atan(disturbance) * 180/pi;
% dist_q     = K_q * atan(disturbance)



%% Control Anticipation:

CAP = g*wn_sp^2 * T_theta2/V;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %% ------------------------- Plotting ------------------------------

plotting = false;

 if plotting == true
     output_names = ['Pitch Angle - \theta [ \circ ]', "Velocity - V_t [m/s]" ,"Angle of Attack - \alpha [ \circ ]", "Pitch Rate - q [ \circ/s]"];

     t_short_period = 0:dt:10;
     y_short_period = step(sys, t_short_period);

     figure(2)
     for f= 1:4
         subplot(2,2,f);
         plot(t_short_period, y_short_period(:,f), 'b-');
         xline(T_1_2_short_period,'--r','T_{1/2}','LineWidth',1);
         xline(P_short_period,'--k','P','LineWidth',1);
         grid on;
         hold on;
         title(output_names(f));
         xlabel("Time [s]")
         ylabel(output_names(f));
     end
     sgtitle('Short Period')


     t_phugoid = 0:dt:800;
     y_phugoid = step(sys, t_phugoid);

     figure(3)


     for f= 1:4
         subplot(2,2,f);
         plot(t_phugoid, y_phugoid(:,f), 'b-');
         xline(T_1_2_phugoid,'--r','T_{1/2}','LineWidth',1);
         xline(P_phugoid,'--k','P','LineWidth',1);
         grid on;
         hold on;
         title(output_names(f));
         xlabel("Time [s]")
         ylabel(output_names(f));
     end
     sgtitle('Phugoid')
 end

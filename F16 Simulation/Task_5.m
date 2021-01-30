A_long_initial= [0.000e+00    9.000e+02    0.000e+00   -9.000e+02    0.000e+00    0.000e+00    0.000e+00; 
                0.000e+00    0.000e+00    0.000e+00    0.000e+00    1.000e+00    0.000e+00    0.000e+00; 
                2.420e-04   -3.217e+01   -1.718e-02    6.397e+01    5.824e-01    1.570e-03    5.250e-01; 
                1.118e-06   -1.748e-13   -7.940e-05   -1.349e+00    9.322e-01   -3.239e-09   -2.833e-03; 
                2.235e-20    0.000e+00   -1.586e-18   -5.916e+00   -1.819e+00    0.000e+00   -4.351e-01; 
                0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00   -1.000e+00    0.000e+00; 
                0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00   -2.020e+01];
      
%Eliminating h (the first state in the original A matrix)

A_long_initial_corrected= [A_long_initial(2, 2:end);
                           A_long_initial(3, 2:end);
                           A_long_initial(4, 2:end);
                           A_long_initial(5, 2:end);
                           A_long_initial(6, 2:end);
                           A_long_initial(7, 2:end)];                
               

%% -------------- Eliminating elevator actuator d_e and engine dynamics d_t
%from A and keep only u_el as input -------------------------------

A_long_a_c= [A_long_initial_corrected(1,1:4);
             A_long_initial_corrected(2,1:4);
             A_long_initial_corrected(3,1:4);
             A_long_initial_corrected(4,1:4)]
         
B_long_a_c= [A_long_initial_corrected(1:4,6)]

A_complete= [[A_long_a_c; [0 0 0 0]],[B_long_a_c; -20.2]];  


C_long_initial = [1.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    1.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00; 
                  0.000e+00    0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00]; 

C_long_a_c = [C_long_initial(2:end,2), C_long_initial(2:end,3),C_long_initial(2:end,4),C_long_initial(2:end,5)] %corrected for altitude


D_long_initial= [0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00; 
             0.000e+00    0.000e+00]; 
         
D_long_a_c= D_long_initial(2:end,1)
 
sys = ss(A_long_a_c,B_long_a_c,C_long_a_c,D_long_a_c);
 
 
 %% Determining the motion characteristics of the phugoid and short period
 [wn,zeta, poles] = damp(sys)
 
 
 wn_phugoid = wn(1);
 zeta_phugoid = zeta(1);
 T_1_2_phugoid = log(0.5)/real(poles(1));
 P_phugoid = 2*pi/imag(poles(1));
 
 
 wn_short_period= wn(3);
 zeta_short_period= zeta(3);
 T_1_2_short_period = log(0.5)/real(poles(3));
 P_short_period = 2*pi/imag(poles(3));
 
 
 %% ------------------------- Plotting ------------------------------
 dt = 0.001;
 initial_values = [0 , 900, 0 , 0];
  
 output_names = ['Pitch Angle - \theta [ \circ ]', "Velocity - V_t [ft/s]" ,"Angle of Attack - \alpha [ \circ ]", "Pitch Rate - q [ \circ/s]"];

 %% Short Period
 
 t_short_period = 0:dt:10;
 y_short_period = initial_values+ step(sys, t_short_period);
 
 figure(1)
 for f= 1:4
     set(gca,'FontSize',17);
     subplot(2,2,f);
     plot(t_short_period, y_short_period(:,f), 'b-', 'LineWidth',1.1);
     xline(T_1_2_short_period,'--r','T_{1/2}','LineWidth',2, 'FontSize',15);
     xline(P_short_period,'--k','P','LineWidth',2, 'FontSize',15);
     grid on;
     hold on;
     set(gca,'FontSize',17);
     legend('Time Response');
     title(output_names(f));
     xlabel("Time [s]")
     ylabel(output_names(f));
 end
 sgtitle('Short Period','FontSize',25)
 
 %% Phugoid
 
 t_phugoid = 0:dt:800;
 y_phugoid = initial_values + step(sys, t_phugoid);
 
 figure(2)
 for f= 1:4
     set(gca,'FontSize',17);
     subplot(2,2,f);
     plot(t_phugoid, y_phugoid(:,f), 'b-', 'LineWidth',1.1);
     xline(T_1_2_phugoid,'--r','T_{1/2}','LineWidth',2, 'FontSize',15);
     xline(P_phugoid,'--k','P','LineWidth',2, 'FontSize',15);
     grid on;
     hold on;
     set(gca,'FontSize',17);
     legend('Time Response');
     title(output_names(f));
     xlabel("Time [s]")
     ylabel(output_names(f));
 end
 sgtitle('Phugoid','FontSize',25)
 

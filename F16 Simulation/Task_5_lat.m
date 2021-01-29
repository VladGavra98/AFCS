A_lat_initial = [0.000e+00    0.000e+00    0.000e+00    0.000e+00    1.000e+00    1.857e-03    0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    1.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00   -1.718e-02    3.840e-14    0.000e+00    0.000e+00    1.570e-03    0.000e+00    0.000e+00; 
                 3.574e-02    0.000e+00    4.741e-20   -4.269e-01    7.287e-04   -9.945e-01    0.000e+00    3.639e-04    1.068e-03; 
                 0.000e+00    0.000e+00    5.372e-19   -6.021e+01   -4.975e+00    6.524e-01    0.000e+00   -1.676e+00    2.947e-01; 
                 0.000e+00    0.000e+00   -1.639e-17    2.223e+01    1.408e-02   -6.579e-01    0.000e+00   -1.078e-01   -1.769e-01; 
                 0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00   -1.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00   -2.020e+01    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00   -2.020e+01] 

%% ------- Eliminating engine dynamics and the other states that are not of interest ----             

roll_angle_phi      = [A_lat_initial(1,1);
                       A_lat_initial(4:6,1)];
                   
sideslip_angle_beta = [A_lat_initial(1,4);
                       A_lat_initial(4:6,4)];
                   
roll_rate_p         = [A_lat_initial(1,5);
                       A_lat_initial(4:6,5)];
                   
yaw_rate_r          = [A_lat_initial(1,6);
                       A_lat_initial(4:6,6)];
                 
                  
%% ------------- Generating A_lat_a_c ----------

A_lat_a_c = [roll_angle_phi, sideslip_angle_beta, roll_rate_p, yaw_rate_r]

%% ------------- Generating B_lat_a_c ----------
aileron_deflection    = [A_lat_initial(1,8);
                       A_lat_initial(4:6,8)];
                   
rudder_deflection     = [A_lat_initial(1,9);
                       A_lat_initial(4:6,9)];
                   
B_lat_a_c = [aileron_deflection, rudder_deflection]


%% ------ Following the same principle to obtain C_lat_a_c and D_lat_a_c----
               
C_lat_initial = [5.730e+01    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    1.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00    0.000e+00    0.000e+00    5.730e+01    0.000e+00    0.000e+00    0.000e+00]; 

C_lat_a_c = [C_lat_initial(1,1), C_lat_initial(1,4:6);
             C_lat_initial(4,1), C_lat_initial(4,4:6);
             C_lat_initial(5,1), C_lat_initial(5,4:6);
             C_lat_initial(6,1), C_lat_initial(6,4:6)];
         
D_lat_initial = [0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00; 
                 0.000e+00    0.000e+00    0.000e+00]; 
             
D_lat_a_c = [D_lat_initial(3:end,2), D_lat_initial(3:end,3)];
             
sys=ss(A_lat_a_c, B_lat_a_c, C_lat_a_c, D_lat_a_c);


%% --------------- Determining the lateral eigenmotions characteristics --------
[wn,zeta,poles] = damp(sys)
 
%% ---------------- Dutch Roll ---------------
 
 wn_dutch_roll = wn(2);
 zeta_dutch_roll = zeta(2);
 T_1_2_dutch_roll = log(0.5)/real(poles(2));
 P_dutch_roll = 2*pi/imag(poles(2));
 
 
 %% --------------- Aperiodic Roll -----------
 
 wn_aperiodic_roll= wn(4);
 zeta_aperiodic_roll= zeta(4);
 T_1_2_aperiodic_roll = log(0.5)/real(poles(4))
 tau_aperiodic_roll = 1.443*T_1_2_aperiodic_roll;
 
 %% --------------- Spiral -----------
 
 wn_spiral= wn(1)
 zeta_spiral= zeta(1)
 T_1_2_spiral = log(0.5)/real(poles(1))
 tau_spiral = 1.443*T_1_2_spiral
 
 
 %% ------------- Plotting -------------
 
 dt = 0.001;
 
 t_dutch = 0:dt:25;
 y_dutch= step(sys(:,2), t_dutch);
 
 output_names = ["Roll Angle - \phi [ \circ ]", "Side-Slip Angle - \beta [ \circ ]", "Roll Rate - p [\circ/s]", "Yaw Rate - r [\circ/s]"];

 t_aroll = 0:dt:10;
 y_aroll = step(sys(:,1), t_aroll);
 
 t_spiral = 0:dt:300;
 y_spiral = step(sys(:,[1,2]), t_spiral);
 
 figure(1)
for f = 1:4
    set(gca,'FontSize',17);
    subplot(2, 2, f);
    plot(t_spiral, y_spiral(:, f), 'b-', 'LineWidth',1.1);
    %xline(T_1_2_spiral,'--r','T_{1/2}','LineWidth',1);
    xline(tau_spiral,'--r','\tau','LineWidth', 2,  'FontSize',20);
    grid on;
    hold on;
    set(gca,'FontSize',17);
    legend('Time Response', "Time Constant");
    title(output_names(f));
    ylabel(output_names(f));
    xlabel("Time [s]");
end
sgtitle("Spiral", 'FontSize', 25)


figure(2)
for f = 1:4
    set(gca,'FontSize',17);
    subplot(2, 2, f);
    plot(t_aroll, y_aroll(:, f), 'b-','LineWidth',1.1);
    %xline(T_1_2_aperiodic_roll,'--r','T_{1/2}','LineWidth',1);
    xline(tau_aperiodic_roll,'--r','\tau','LineWidth',2,  'FontSize',20);
    grid on;
    hold on;
    set(gca,'FontSize',17);
    legend('Time Response', "Time Constant");
    title(output_names(f));
    ylabel(output_names(f));
    xlabel("Time [s]");
end
sgtitle("Aperiodic Roll",'FontSize', 25);


figure(3)
for f = 1:4
    set(gca,'FontSize',17);
    subplot(2, 2, f);
    plot(t_dutch, y_dutch(:, f), 'b-', 'LineWidth',1.1); 
    xline(T_1_2_dutch_roll,'--r','T_{1/2}','LineWidth',2,  'FontSize',15);
    xline(P_dutch_roll,'--k','P','LineWidth',2,  'FontSize',15);
    grid on;
    hold on;
    set(gca,'FontSize',17);
    legend('Time Response');
    title(output_names(f));
    ylabel(output_names(f));
    xlabel("Time [s]");
end
sgtitle("Dutch Roll",'FontSize', 25);

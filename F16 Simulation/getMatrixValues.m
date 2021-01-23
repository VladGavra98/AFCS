% constants
minEngine = 1000;
maxEngine = 19000;
minElevator = -25;
maxElevator = 25;


% trim values
trCost   = 4.7464e-30;
trThrust = 2826.8165;
trElev   = -4.1891;
trAil    = -1.9926e-15;
trRud    = 1.2406e-14;
trAlpha  = 10.4511;
trDLEF   = 0;
trVel    = 300;

% initial conditions
initial = [0, 0, 0, 0, 0];

% state space matrix
SS = load('f16long5000ft300fts.mat', 'SS_long_lo');
SS_long_lo = SS.SS_long_lo;
A = SS_long_lo.A(1:5,1:5);
B = SS_long_lo.A(1:5,6:7);
C = SS_long_lo.C(:,1:5);
D = SS_long_lo.D;

% set state space block parameters
set_param('glideslope/AircraftSS','A','A','B','B','C','C','D','D','InitialCondition','initial')

% saturation limits
minEngineSat = -(trThrust - minEngine)
maxEngineSat = maxEngine - trThrust

minElevSat = -(trElev - minElevator)
maxElevSat = maxElevator - trElev

% airport settings
horDistInitial = 38162.3 + 300*10


% fileID = fopen('stateSpaceCoefficients.txt','w');
% fprintf(fileID,'%6s\n','A');
% fprintf(fileID,'%6s\n','[');
% fprintf(fileID,'%12.8f, %12.8f, %12.8f, %12.8f, %12.8f, %12.8f, %12.8f;\n',A);
% fprintf(fileID,'%6s\n',']');
% 
% fprintf(fileID,'%6s\n','B');
% fprintf(fileID,'%6s\n','[');
% fprintf(fileID,'%12.8f, %12.8f;\n',B);
% fprintf(fileID,'%6s\n',']');
% 
% fprintf(fileID,'%6s\n','C');
% fprintf(fileID,'%6s\n','[');
% fprintf(fileID,'%12.8f, %12.8f, %12.8f, %12.8f, %12.8f, %12.8f, %12.8f;\n',C);
% fprintf(fileID,'%6s\n',']');
% 
% fprintf(fileID,'%6s\n','D');
% fprintf(fileID,'%6s\n','[');
% fprintf(fileID,'%12.8f, %12.8f;\n',D);
% fprintf(fileID,'%6s\n',']');
% 
% fclose(fileID);
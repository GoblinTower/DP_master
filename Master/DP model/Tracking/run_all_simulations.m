% Run all simulations
% intended for saving time
clear, clc, close all;

external_scenario = 'run_all_scenarios_tracking';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Run with disturbance %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run_with_disturbance = true;

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 1/8');
run 'LPV_simple_tracking';

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 2/8');
run LPV_trajectory_tracking.m;

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 3/8');
run 'LQ_simple_tracking';

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 4/8');
run 'LQ_trajectory_tracking';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Run without disturbance %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run_with_disturbance = false;

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 5/8');
run 'LPV_simple_tracking';

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 6/8');
run LPV_trajectory_tracking.m;

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 7/8');
run 'LQ_simple_tracking';

clearvars -except external_scenario run_with_disturbance;
close all;
disp('Running simulation 8/8');
run 'LQ_trajectory_tracking';

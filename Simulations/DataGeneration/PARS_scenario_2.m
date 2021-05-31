clear all; close all; clc;

scenario_id = "PARS Scenario 2";
scenario_file_name = "Simulations/GeneratedData/PARS_scenario_02.mat";
scenario_t = "GNSS-denied PARS-based collaborative navigation - Scenario 2";

% Time, N, E, D, Yaw, Pitch, Roll

constraints = ...
            [0 100 100 0 90 0 0;
             90 400 150 0 90 0 0;
             270 650 250 0 0 0 5;
             450 1100 440 0 0 0 -5;
             540 1400 500  0 -45 0 0;
             900 3000 500 0 90 0 0];
  
         
constraints2 = ...
            [0   -10000 10000 0 0 0 0; 
             900 -10000 10000 0 0 0 0];
         
constraints3 = ...
            [0   -10005 10005 0 0 0 0; 
             900 -10005 10005 0 0 0 0];
   
constraints4 = ...
            [0   -10010 10010 0 0 0 0; 
             900 -10010 10010 0 0 0 0];        
  
 
% Generate data from waypoint trajectory
[N, Ts, position, vel, orient] = generate_waypoint_trajectory(constraints);
[~, ~, pos2, vel2, orient2] = generate_waypoint_trajectory(constraints2);
[~, ~, pos3, vel3, orient3] = generate_waypoint_trajectory(constraints3);
[~, ~, pos4, vel4, orient4] = generate_waypoint_trajectory(constraints4);

t = 0:Ts:(N-1)*Ts;

% Interpolating data
[p, v, q, a, omega] = ...
    interpolate_simulation_data(N, Ts, position, vel, orient);

[p2, v2, q2, a2, omega2] = ...
    interpolate_simulation_data(N, Ts, pos2, vel2, orient2);

[p3, v3, q3, a3, omega3] = ...
    interpolate_simulation_data(N, Ts, pos3, vel3, orient3);

[p4, v4, q4, a4, omega4] = ...
    interpolate_simulation_data(N, Ts, pos4, vel4, orient4);

save(scenario_file_name, "p", "p2","p3","p4", "v", "v2","v3","v4","q","q2", "q3","q4", "a", "a2","a3","a4","omega","omega2","omega3","omega4", "Ts", "N", "t")



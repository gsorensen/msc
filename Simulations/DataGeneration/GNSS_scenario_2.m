clear all; close all; clc;

scenario_id = "GNSS Scenario 2";
scenario_file_name = "Simulations/GeneratedData/GNSS_scenario_02.mat";
scenario_t = "GNSS-based single agent navigation – Scenario 2";
 
% Time, N, E, D, Yaw, Pitch, Roll
constraints = ...
            [   0 100   10 0 -90 0 3;
               50 100 -100 0 -90 0 1;
              100 100 -175 0 -90 0 3;
              150 200 -250 0   0 0 0;
              200 300 -175 0  90 0 3;
              250 300 -100 0  90 0 1;
              300 300   10 0  90 0 3];

% Generate data from waypoint trajectory
[N, Ts, position, vel, orient] = generate_waypoint_trajectory(constraints);
t = 0:Ts:(N-1)*Ts;


% Interpolating data
[p, v, q, a, omega] = ...
    interpolate_simulation_data(N, Ts, position, vel, orient);
save(scenario_file_name, "p", "v", "q", "a", "omega", "Ts", "N", "t")
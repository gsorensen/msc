clear all; close all; clc;

scenario_id = "GNSS Scenario 3";
scenario_file_name = "Simulations/GeneratedData/GNSS_scenario_03.mat";
scenario_t = "GNSS-based single agent navigation – Scenario 3";
 
% Time, N, E, D, Yaw, Pitch, Roll
constraints = ...
            [ 0 40  100 0 90 0 0;
             30 70 100 0 90 0 0
             60 100 100 0 90 0 0;
             85 180 160 0 0 0 3;
             115 340 40  0 45 0 -3;
             165 340 160 0 -45 0 3;
             190 180 40  0 45 0 -3;
             240 110 110 0 -45 0 0];

% Generate data from waypoint trajectory
[N, Ts, position, vel, orient] = generate_waypoint_trajectory(constraints);
t = 0:Ts:(N-1)*Ts;


% Interpolating data
[p, v, q, a, omega] = ...
    interpolate_simulation_data(N, Ts, position, vel, orient);
save(scenario_file_name, "p", "v", "q", "a", "omega", "Ts", "N", "t")

clear all; close all; clc;

scenario_id = "GNSS Scenario 1";
scenario_file_name = "Simulations/GeneratedData/GNSS_scenario_01.mat";
scenario_t = "GNSS-based single agent navigation – Scenario 1";

% Time, N, E, D, Yaw, Pitch, Roll

constraints = ...
            [0 100 100 0 90 0 0;
             90 400 150 0 90 0 0;
             270 650 250 0 0 0 5;
             450 1100 440 0 0 0 -5;
             540 1400 500  0 -45 0 0;
             900 3000 500 0 90 0 0];


        
% Generate data from waypoint trajectory
[N, Ts, position, vel, orient] = generate_waypoint_trajectory(constraints);
t = 0:Ts:(N-1)*Ts;

% Interpolating data
[p, v, q, a, omega] = ...
    interpolate_simulation_data(N, Ts, position, vel, orient);

save(scenario_file_name, "p", "v", "q", "a", "omega", "Ts", "N", "t")

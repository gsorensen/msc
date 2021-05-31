function [N, Ts, position, vel, orient] = generate_waypoint_trajectory(constraints)
% Based off of MATLAB documentation on Waypoint Trajectory Generator
% https://www.mathworks.com/help/fusion/ref/waypointtrajectory-system-object.html
trajectory = waypointTrajectory(constraints(:,2:4), ...
    'TimeOfArrival',constraints(:,1), ...
    'Orientation',quaternion(constraints(:,5:7),'eulerd','ZYX','frame'),...
    'AutoBank', true);
tInfo = waypointInfo(trajectory);

position = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);
orient = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,'quaternion');
vel = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);
count = 1;
while ~isDone(trajectory)
   [pos,orient(count),vel(count,:),~,~] = trajectory();
   position(count,:) = pos;
   count = count + 1;
end

N = count - 1;
Ts = 1 / trajectory.SampleRate;
end
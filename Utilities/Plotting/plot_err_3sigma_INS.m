function plot_err_3sigma_INS(fignum,t, x, x_t, P, identifier, case_description)

figure(fignum);clf;hold on; 
s1=subplot(5,3,1); s2=subplot(5,3,2); s3=subplot(5,3,3);
s4=subplot(5,3,4); s5=subplot(5,3,5); s6=subplot(5,3,6);
s7=subplot(5,3,7); s8=subplot(5,3,8); s9=subplot(5,3,9);
s10=subplot(5,3,10); s11=subplot(5,3,11); s12=subplot(5,3,12);
s13=subplot(5,3,13); s14=subplot(5,3,14); s15=subplot(5,3,15);
sgtitle(sprintf("Estimation error and 3-sigma-bounds | %s | %s", identifier, case_description)); 
hold(s1,"on");
hold(s2,"on");
hold(s3,"on");
hold(s4,"on");
hold(s5,"on");
hold(s6,"on");
hold(s7,"on");
hold(s8,"on");
hold(s9,"on");
hold(s10,"on");
hold(s11,"on");
hold(s12,"on");
hold(s13,"on");
hold(s14,"on");
hold(s15,"on");

xlabel(s1, "Time [s]");  ylabel(s1, "North [m]");
xlabel(s2, "Time [s]");  ylabel(s2, "East [m]");
xlabel(s3, "Time [s]");  ylabel(s3, "Down [m]");
xlabel(s4, "Time [s]");  ylabel(s4, "North [m/s]");
xlabel(s5, "Time [s]");  ylabel(s5, "East [m/s]");
xlabel(s6, "Time [s]");  ylabel(s6, "Down [m/s]");
xlabel(s7, "Time [s]");  ylabel(s7, "Roll [deg]");
xlabel(s8, "Time [s]");  ylabel(s8, "Pitch [deg]");
xlabel(s9, "Time [s]");  ylabel(s9, "Yaw [deg]");
xlabel(s10, "Time [s]"); ylabel(s10, "x [m/s^2]");
xlabel(s11, "Time [s]"); ylabel(s11, "y [m/s^2]");
xlabel(s12, "Time [s]"); ylabel(s12, "z [m/s^2]");
xlabel(s13, "Time [s]"); ylabel(s13, "r [deg/s]");
xlabel(s14, "Time [s]"); ylabel(s14, "p [deg/s]");
xlabel(s15, "Time [s]"); ylabel(s15, "q [deg/s]");

% Position
title(s2, "Position");
Pxx = 3.*(P(1,1,:)).^(1/2);
Pyy = 3.*(P(2,2,:)).^(1/2);
Pzz = 3.*(P(3,3,:)).^(1/2);
plot(s1, t, x_t(1,:)-x(1,:));
plot(s1, t, Pxx(:), '--');
plot(s1,t, -Pxx(:), '--');

axis(s1, [min(t) max(t) min(-Pxx)-1 max(Pxx)+1]); 

plot(s2, t, x_t(2,:)-x(2,:));
plot(s2, t, Pyy(:), '--');
plot(s2,t, -Pyy(:), '--');
axis(s2, [min(t) max(t) min(-Pyy)-1 max(Pyy)+1]);

plot(s3, t, x_t(3,:)-x(3,:));
plot(s3, t, Pzz(:), '--');
plot(s3,t, -Pzz(:), '--');
axis(s3, [min(t) max(t) min(-Pzz)-1 max(Pzz)+1]);


% Velocity
title(s5, "Velocity");
Pxx = 3.*(P(4,4,:)).^(1/2);
Pyy = 3.*(P(5,5,:)).^(1/2);
Pzz = 3.*(P(6,6,:)).^(1/2);
plot(s4, t, x_t(4,:)-x(4,:));
plot(s4, t, Pxx(:), '--');
plot(s4,t, -Pxx(:), '--');
axis(s4, [min(t) max(t) min(-Pxx)-1 max(Pxx)+1]);
plot(s5, t, x_t(5,:)-x(5,:));
plot(s5, t, Pyy(:), '--');
plot(s5,t, -Pyy(:), '--');
axis(s5, [min(t) max(t) min(-Pyy)-1 max(Pyy)+1]);
plot(s6, t, x_t(6,:)-x(6,:));
plot(s6, t, Pzz(:), '--');
plot(s6,t, -Pzz(:), '--');
axis(s6, [min(t) max(t) min(-Pzz)-1 max(Pzz)+1]);

% Attitude
%t = 1 : N;
eulers_t = zeros(size(x(1:3,:)));
eulers = zeros(size(x(1:3,:)));
for i = 1 : size(x,2)
[phi_t, theta_t, psi_t] = q2euler(x_t(7:10,i));
[phi, theta, psi] = q2euler(x(7:10,i));
eulers_t(:,i) = [phi_t; theta_t; psi_t];
eulers(:,i) = [phi; theta; psi];
end
Pxx = 3.*(P(7,7,:)).^(1/2);
Pyy = 3.*(P(8,8,:)).^(1/2);
Pzz = 3.*(P(9,9,:)).^(1/2);
title(s8, "Attitude");
plot(s7, t, (180/pi).*ssa(eulers_t(1,:)-eulers(1,:)));
plot(s7, t, (180/pi).*Pxx(:), '--');
plot(s7,t, -(180/pi).*Pxx(:), '--');
axis(s7, [min(t) max(t) min(-(180/pi).*Pxx)-1 max((180/pi).*Pxx)+1]);
plot(s8, t, (180/pi).*ssa(eulers_t(2,:)-eulers(2,:)));
plot(s8, t, (180/pi).*Pyy(:), '--');
plot(s8,t, -(180/pi).*Pyy(:), '--');
axis(s8, [min(t) max(t) min(-(180/pi).*Pyy)-1 max((180/pi).*Pyy)+1]);
plot(s9, t, (180/pi).*ssa(eulers_t(3,:)-eulers(3,:)));
plot(s9, t, (180/pi).*Pzz(:), '--');
plot(s9,t, -(180/pi).*Pzz(:), '--');
axis(s9, [min(t) max(t) min(-(180/pi).*Pzz)-1 max((180/pi).*Pzz)+1]);

% Accelerometer bias
Pxx = 3.*(P(10,10,:)).^(1/2);
Pyy = 3.*(P(11,11,:)).^(1/2);
Pzz = 3.*(P(12,12,:)).^(1/2);
title(s11, "Accelerometer bias");
hold(s1, "on");
plot(s10, t, x_t(11,:)-x(11,:));
plot(s10, t, Pxx(:), '--');
plot(s10,t, -Pxx(:), '--');
axis(s10, [min(t) max(t) min(-Pxx)-1 max(Pxx)+1]);
hold(s11, "on");
plot(s11, t, x_t(12,:)-x(12,:));
plot(s11, t, Pyy(:), '--');
plot(s11,t, -Pyy(:), '--');
axis(s11, [min(t) max(t) min(-Pyy)-1 max(Pyy)+1]);
hold(s12, "on");
plot(s12, t, x_t(13,:)-x(13,:));
plot(s12, t, Pzz(:), '--');
plot(s12,t, -Pzz(:), '--');
axis(s12, [min(t) max(t) min(-Pzz)-1 max(Pzz)+1]);


% Gyro bias
Pxx = 3.*(P(13,13,:)).^(1/2);
Pyy = 3.*(P(14,14,:)).^(1/2);
Pzz = 3.*(P(15,15,:)).^(1/2);
title(s14, "Gyroscope bias");
hold(s13, "on");
plot(s13, t, (180/pi).*(x_t(14,:)-x(14,:)));
plot(s13, t, (180/pi).*Pxx(:), '--');
plot(s13,t, -(180/pi).*Pxx(:), '--');
axis(s13, [min(t) max(t) (180/pi).*min(-Pxx(10:end))-1 (180/pi).*max(Pxx(10:end))+1]);
hold(s14, "on");
plot(s14, t, (180/pi).*(x_t(15,:)-x(15,:)));
plot(s14, t, (180/pi).*Pyy(:), '--');
plot(s14, t, -(180/pi).*Pyy(:), '--');
axis(s14, [min(t) max(t) (180/pi).*min(-Pyy(10:end))-1 (180/pi).*max(Pyy(10:end))+1]);
hold(s15, "on");
plot(s15, t, (180/pi).*(x_t(16,:)-x(16,:)));
plot(s15, t, (180/pi).*Pzz(:), '--');
plot(s15,t, -(180/pi).*Pzz(:), '--');
axis(s15, [min(t) max(t) (180/pi).*min(-Pzz(10:end))-1 (180/pi).*max(Pzz(10:end))+1]);
end


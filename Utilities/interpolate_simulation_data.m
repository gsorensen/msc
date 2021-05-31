function [p, v, q, a, omega_n] = interpolate_simulation_data(N, Ts, position, vel, orient)
% Angular veloctiy and attitude following SLERP in Sola
omega = zeros(3,N);
q_gen = orient.compact';

for i = 1 : N - 1
    q0 = q_gen(:,i);
    q1 = q_gen(:,i+1);
    q0c = [q0(1); -q0(2:4)]; 
    dq = quatprod(q0c, q1);
    w_norm = acos(dq(1))*2;
    if w_norm > 1e-10
        u = dq(2:4) / sin(w_norm/2);
        w = u*w_norm;
        omega(:,i) = w/Ts;    
    end
end

q = zeros(4,N);
q(:,1) = q_gen(:,1);

for i = 1 : N - 1
    w = omega(:,i)*Ts;
    w_norm = norm(w);
    if w_norm > 1e-10
        u = w / w_norm;
        dq =  [cos(w_norm/2); sin(w_norm/2)*u];
    else
        dq = [1 0 0 0]';
    end
    
    q(:,i+1) = quatprod(q(:,i), dq);
end

% IMU wants NED angular velocity
omega_n = zeros(3, N);

for i = 1 : N 
   omega_n(:, i) = Rquat(q(:,i)) * omega(:, i); 
end


% Velocity generation - Groves 2013 Appendix J.3.2
v = zeros(3,N);
v(:,1) = vel(1,:)';

for i = 1 : N - 1
    v0 = v(:, i);
    p0 = position(i, :)';
    p1 = position(i+1,:)';
    v1 = (2 / Ts) * (p1 - p0) - v0;
    v(:,i + 1) = v1;
end

% Acceleration generation - Groves 2013 Appendix J.3.2
a = zeros(3,N);

for i = 1 : N - 1
   v0 = v(:, i);
   v1 = v(:, i + 1);
   a(:,i) = (v1 - v0) / Ts;
end

p = position(1:N,:)';
end

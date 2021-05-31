classdef Agent3DNominal
    %AGENT3DNOMINAL 
    %   Implementation of the discrete-time nominal state-space model 
    %   defined in section 4.3.1 in the thesis.
    
    properties
        Ts
    end
    
    methods
        function obj = Agent3DNominal(Ts)
            %AGENT3DNOMINAL Construct an instance of this class
            %   Detailed explanation goes here
            obj.Ts = Ts;
        end
        
        function [x,a_n, w_imu] = propagate(obj, x_k, u_k)
            % x_k State at time step k
            % u_k = [f_imu, w_imu]' Input from IMU at time step k
            % time step k
            
            % Returns x as the next discrete-time nominal state
            
            % State
            p = x_k(1:3);
            v = x_k(4:6);
            q = x_k(7:10);
            b_acc = x_k(11:13);
            b_gyro = x_k(14:16);

            % Bias compansated input
            f_imu = u_k(1:3) - b_acc;
            w_imu = u_k(4:6) - b_gyro;
            
            R = Rquat(q);
 
            a_n = (R * f_imu + [0 0 -9.8100]') ;         
            norm_w = norm(w_imu);

            if norm_w > 1e-10
                q_rot = [cos(norm_w*obj.Ts/2); 
                     w_imu/norm_w * sin(norm_w*obj.Ts/2)];
            else
                q_rot = [1 0 0 0]';
            end
            
            x = zeros(size(x_k));
            x(1:3)  = p + obj.Ts .* v + (obj.Ts^2/2) .* a_n;
            x(4:6)  = v + obj.Ts .* a_n;
            x(7:10) = quatprod(q, q_rot);
            x(7:10) = x(7:10) / norm(x(7:10)); % Normalisation
            x(11:13) = b_acc;
            x(14:16) = b_gyro;
        end
    end
end


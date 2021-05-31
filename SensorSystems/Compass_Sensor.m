classdef Compass_Sensor < handle   
    % Implenetation of the measurement model in Section 4.4.4
    properties
        mu
        sigma
        pregen
        v
        idx
    end
    
    methods
        function obj = Compass_Sensor(mu, sigma)
            % Initialises a compass to received 1D heading measurements.
            % Give the standard deviation in degrees.
            obj.mu = mu;
            obj.sigma = deg2rad(sigma);
            obj.pregen = 0;
        end
        
        function set_data(obj, v)
           obj.pregen = 1;
           obj.v = v;
           obj.idx = 1;
        end
        
        function [y, h, H, R] = get_heading_measurement(obj, q, q_t)
            [~,~, psi_t] = q2euler(q_t);
            [~,~, psi] = q2euler(q);     
            
            if obj.pregen == 0
                w = obj.mu + obj.sigma.^2 * randn();
                %w = 0;
            else
                w = obj.v(:, obj.idx);
                obj.idx = obj.idx + 1;
            end
            
            y = ssa(psi_t + w);
            
            h = ssa(psi);
            
            H_yaw = @(qw,qx,qy,qz)[0,-(((2*qw*qx-2*qy*qz)/(2*qy^2+2*qz^2-1)-((2*qw*qy+2*qx*qz)*(2*qw*qz+2*qx*qy))/(2*qy^2+2*qz^2-1)^2)*(2*qy^2+2*qz^2-1)^2)/((2*qy^2+2*qz^2-1)^2+(2*qw*qz+2*qx*qy)^2),(((2*qx^2+2*qz^2-1)/(2*qy^2+2*qz^2-1)+((2*qw*qz-2*qx*qy)*(2*qw*qz+2*qx*qy))/(2*qy^2+2*qz^2-1)^2)*(2*qy^2+2*qz^2-1)^2)/((2*qy^2+2*qz^2-1)^2+(2*qw*qz+2*qx*qy)^2)];

            H = [zeros(1,6) H_yaw(q(1),q(2),q(3),q(4)) zeros(1,6)];
            
            R = deg2rad(0.02).^2 ;
        end
    end
end


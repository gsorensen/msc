classdef GNSS_Sensor < handle
    % Implementation of the measurement model in section 4.4.1
    properties
       mu
       sigma 
       pregen
       v
       idx
    end
    
    methods
        function obj = GNSS_Sensor(mu, sigma)
            % Initialises a GNSS sensor with the given mu and standard
            % deviation
            % mu 3x1 mu vector
            % sigma  3x1 sigma vector
            obj.mu = mu;
            obj.sigma = sigma;
            obj.pregen = 0;
        end
        
        function set_data(obj, v)
           if size(v, 1) == 3
              obj.pregen = 1;
              obj.v = v;
              obj.idx = 1;
           end
        end
        
        function [y, h, H, R] = get_position_measurement(obj, pos_i)
            % Get a simulated GNSS measurement from local position 
            %
            % Inputs
            % pos_i 2D/3D position vector of the local agent
            %
            % Outputs
            % y The position measurement from GNSS in NED-frame 
            % h The position measurement model 
            % H The measurement Jacobian / observation matrix
            % R The measurement noise covariance matrix

            % Measurement noise mu/bias and standard deviation
            % Check whether we're dealing with 2D or 3D
            dim_pos = size(pos_i, 1);

            % Measurement noise
            if obj.pregen == 0
                vk = obj.mu(1:dim_pos) + obj.sigma(1:dim_pos) .* randn(dim_pos,1);
                %vk = [0 0 0]';
            else
                vk = obj.v(:,obj.idx);
                obj.idx = obj.idx + 1;
            end

            % Create the position measurement
            y = pos_i + vk;

            % Create the measurement Jacobian, since this is a linear setup
            H = [eye(dim_pos) zeros(dim_pos, 15 - dim_pos)];

            % Create the measurement model which is simply the state multiplied by the measurement Jacobian
            h = @(x) x(1:3);
            
            % The measurement noise covariance
            R = diag(([1.5 1.5 10]).^2);
        end
    end
end
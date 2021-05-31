classdef PARS_Sensor < handle
    % PARS_SENSOR
    % Implementation of the PARS measurement model defined in section 4.4.2
    % in the thesis.
    properties
        mu_rho
        mu_Psi
        mu_alpha
        sigma_rho
        sigma_Psi
        sigma_alpha
        pregen
        v
        idx
    end
    
    methods
        function obj = PARS_Sensor(mu_rho, mu_Psi, mu_alpha, sigma_rho, sigma_Psi, sigma_alpha)
            obj.mu_rho = mu_rho;
            obj.mu_Psi = mu_Psi;
            obj.mu_alpha = mu_alpha;
            obj.sigma_rho = sigma_rho;
            obj.sigma_Psi = sigma_Psi;
            obj.sigma_alpha = sigma_alpha;
            obj.pregen = 0;
        end
        
        function set_data(obj, v)
           if size(v, 1) == 3
              obj.pregen = 1;
              obj.v = v;
              obj.idx = 1;
           end
        end

        function increment_pregen_idx(obj)
           obj.idx = obj.idx + 1; 
        end
        
        function [z_p_n, h, H, R, H_c] = get_position_measurement(obj, p_i, p_j, R_r_n, ~)
            [z_rho,  R_rho] = get_range_measurement(obj,p_i, p_j);
            [z_Psi, R_Psi] = get_azimuth_angle_measurement(obj,p_i, p_j);
            [z_alpha,  R_alpha] = get_elevation_angle_measurement(obj,p_i, p_j);
            
            if obj.pregen == 1
               %obj.idx = obj.idx + 1; 
            end
            
            b_Psi = exp(-R_Psi/2);
            b_alpha = exp(-R_alpha/2);
            
            x = (1/b_Psi) * (1/b_alpha) * cos(z_Psi) * cos(z_alpha);
            y = sin(z_Psi) * cos(z_alpha);
            z = -sin(z_alpha);
            
            z_p_n = R_r_n * (z_rho.*[x y z]');
            
            % If we're utilising Schmidt Kalman, then the measurement model
            % is a function of both local and remote agent position
            if nargin == 4
                h = @(p) p - p_j; 
                H_c = zeros(3);
            else
                h = @(p,p_c) p - p_c;
                H_c = -eye(3);
            end
            
            H = [eye(3) zeros(3,12)];
            
            R_s = diag([R_rho R_Psi R_alpha]);
            
            b_den = b_Psi * b_alpha; 
            
            % Method described in Kristoffer Gryte Ph.D 2020. See thesis
            % references for citation, but the Ph.D thesis in question is
            % found here: 
            % https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2657113
            m11 = cos(z_Psi) * cos(z_alpha) / b_den;
            m12 = -z_rho * cos(z_alpha) * sin(z_Psi) / b_den;
            m13 = -z_rho * cos(z_Psi) * sin(z_alpha) / b_den;
            m21 = cos(z_alpha) * sin(z_Psi) / b_den;
            m22 = z_rho * cos(z_Psi) * cos(z_alpha) / b_den;
            m23 = -z_rho * sin(z_Psi) * sin(z_alpha) / b_den;
            m31 = -sin(z_alpha) / b_alpha;
            m32 = 0;
            m33 = - z_rho * cos(z_alpha) / b_alpha;
            
            M = [m11 m12 m13; 
                 m21 m22 m23;
                 m31 m32 m33];
            
            R = R_r_n * M * R_s * M' * R_r_n';
            
            function [z_alpha, R] = get_elevation_angle_measurement(obj, p_i, p_j)
            % Gets the elevation angle measurement between a local agent and a remote agent with a given measurement noise
            % Also gets measurement model and measurement Jacobian for use with e.g. EKF
            %
            % Inputs
            % p_i   - Position vector of a local agent's position in NED frame (radio frame) (3x1)
            % p_j   - Position vector of a remote agent's position in NED frame (radio frame)(3x1)
            % 
            % Outputs
            % alpha  - The noisy azimuth elevation measurement that has had ssa applied on it
            % h @(x) - Function handle with the azimuth angle measurement model, assuming that position is first in the state
            % H @(x) - Function handle with the azimuth Jacobian, assuming that position is first in the state
            dim_pos = size(p_i, 1);
            if dim_pos ~= 3
                error("Position measurement not 3D");
            end
            
            rho_bar = norm(p_i(1:2) - p_j(1:2), 2);
            v_mean = obj.mu_alpha;
            v_std = obj.sigma_alpha;
            
            if obj.pregen == 0   
                vk = v_mean + v_std * randn();
            else
                vk = obj.v(3, obj.idx);
            end
            
            z_alpha = ssa(atan(-(p_i(3) - p_j(3)) / rho_bar) + vk);
            
            R = v_std.^2;
            
            end
            
            function [z_Psi, R] = get_azimuth_angle_measurement(obj, p_i, p_j)
            % Gets the azimuth angle measurement between a local agent and a remote agent with a given measurement noise
            % Also gets measurement model and measurement Jacobian for use with e.g. EKF
            %
            % Inputs
            % p_i   - Position vector of a local agent's position in NED frame (radio frame) (3x1)
            % p_j   - Position vector of a remote agent's position in NED frame (radio frame)(3x1)
            % 
            % Outputs
            % Psi    - The noisy azimuth angle measurement that has had ssa applied on it
            % h @(x) - Function handle with the azimuth angle measurement model, assuming that position is first in the state
            % H @(x) - Function handle with the azimuth Jacobian, assuming that position is first in the state
            
            % Getting the position dimension and check that it's OK
            dim_pos = size(p_i, 1);
            if dim_pos ~= 3
                error("Position measurement not 3D");
            end
            
            v_mean = obj.mu_Psi;
            v_std = obj.sigma_Psi;
            
            % Get Psi and apply smallest signed angle on it
            if obj.pregen == 0
                vk = v_mean + v_std * randn();
            else
                vk = obj.v(2, obj.idx);
            end
            
            Psi_r = atan2(p_i(2) - p_j(2), p_i(1) - p_j(1)) + vk;
            z_Psi = ssa(Psi_r);

            R = v_std.^2;
            end
            
            function [z_rho, R] = get_range_measurement(obj, p_i, p_j)
            % Gets the range measurement between a local agent and a remote agent with a given measurement noise
            % Also gets measurement model and measurement Jacobian for use with e.g. EKF
            %
            % Inputs
            % p_i   - Position vector of a local agent's position in NED frame (radio frame) (3x1)
            % p_j   - Position vector of a remote agent's position in NED frame (radio frame) (3x1)
            % 
            % Outputs
            % rho    - The noisy range measurement 
            % h @(x) - Function handle with the range measurement model, assuming that position is first in the state
            % H @(x) - Function handle with the range Jacobian, assuming that position is first in the state

            % Getting the position dimension and check that it's OK
            dim_pos = size(p_i, 1);
            if dim_pos ~= 3
                error("Position measurement not 3D");
            end
            v_mean = obj.mu_rho;
            v_std = obj.sigma_rho;
            
            % Get the range itself
            if obj.pregen == 0
                vk = v_mean + v_std * randn();
            else
                vk = obj.v(1, obj.idx);
            end

            z_rho = norm(p_i - p_j) + vk;
            
            R = v_std.^2;
        end
        end
    end
end
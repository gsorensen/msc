classdef IMU_Sensor < handle    
    % Implenetation of the measurement model in Section 4.4.3
    properties
        imu
        pregen
        u
        idx
    end
    
    methods
        function obj = IMU_Sensor(Ts)
            % Values from Kongsberg Seatex's MRU 5+ MK II
            % https://www.kongsberg.com/globalassets/maritime/km-products/product-documents/datasheet_mru5plus-mkii.pdf
            obj.imu = imuSensor('accel-gyro','SampleRate', 1/Ts);

            % Accelerometer
            obj.imu.Accelerometer.MeasurementRange = 30;
            obj.imu.Accelerometer.Resolution = 60/2^32;
            obj.imu.Accelerometer.ConstantBias = 80*9.81/1E6; 
            obj.imu.Accelerometer.BiasInstability = 2*9.81/1E6;
            obj.imu.Accelerometer.NoiseDensity = 3.3*9.81/1E6;

            % Gyroscope
            obj.imu.Gyroscope.MeasurementRange = deg2rad(75); 
            obj.imu.Gyroscope.Resolution = deg2rad(150) / 2^32; 
            obj.imu.Gyroscope.NoiseDensity = 0.1*deg2rad(0.006)/3600; 
            obj.imu.Gyroscope.BiasInstability = deg2rad(0.03)/3600;
            obj.imu.Gyroscope.ConstantBias = deg2rad(20)/3600; 

            obj.pregen = 0;
        end
        
        function set_data(obj, u)
           % If IMU data is pregenerated, set it and set the pregen flag, then this will be 
           % used instead of newly generated data
           if size(u,1) == 6
               obj.pregen = 1;
               obj.u = u;
               obj.idx = 1;
           else
               error("Incorrect dimensions, u = [a, w], 6 x N");
           end
        end

        function u_k = get_signal_att(obj,a, w, att)
            if obj.pregen == 1
                u_k = obj.u(:,obj.idx);
                obj.idx = obj.idx + 1;
            else
                % Sign change of a necessary because it should give
                % opposite force
                if size(att,1) == 4
                    [acc_data, gyro_data] = obj.imu(-a, w, quaternion(att(1),att(2),att(3),att(4)));
                else
                    [acc_data, gyro_data] = obj.imu(-a, w, att);
                end
                u_k = [acc_data'; gyro_data'];
            end
        end
   
        function [z_imu, h_imu, H_imu, R_imu] = get_measurement(obj, q_k, u_k)
           % Returns attitude measurements based on accelerometer readings
           norm_u = norm(u_k(1:3));
           z_imu = u_k(1:3) / norm_u; % Normalised accelerometer meas.

           R = Rquat(q_k);
           
           g_n_bar = [0 0 -1]';
           
           h_imu = -R' * g_n_bar; 
           
           H_imu = [zeros(3,6) -Smtrx(R' * g_n_bar) zeros(3,6)]; 
           R_imu = diag([0.03 0.03 0.03]).^2;
        end
    end
end


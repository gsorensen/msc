classdef SESKF < handle
    %SESKF Error-state Kalman filter used for positioning of the agents.
    %Has two modes of operation, ESKF or Schmidt ESKF. Filter algorithm 
    %defined in Section 4.6
    
    properties
        Ts 
        nominal_model 
        error_model
        Q_d
        Q_dc
        F_d
        F_dc
        G_d
        G_dc
    end
    
    methods
        function obj = SESKF(Ts, nominal_model, error_model, Q)
            %SESKF Construct an instance of this class
            obj.Ts = Ts;
            obj.nominal_model = nominal_model;
            obj.error_model = error_model;
            obj.Q_d = Q * Ts; % Use the first order approx.instead of Van Loan
        end
        
        function [x_pred, P_pred] = predict(obj, x, P, u, C)
            if nargin == 4
                % Currently not in a setup where we need the consider model
                
                % State prediction
                x_pred = obj.nominal_model.propagate(x, u);

                % Quaternion normalisation
                x_pred(7:10) = x_pred(7:10) / norm(x_pred(7:10)); 

                % Discretisation necessary for state covariance prediction
                I_15 = eye(15);
                obj.F_d = I_15 + obj.Ts .* obj.error_model.F(x, u);
                obj.G_d = obj.Ts .* obj.error_model.G(x);

                % State covariance prediction
                P_pred = obj.F_d * P * obj.F_d' + obj.G_d * obj.Q_d * obj.G_d';
                
            else
                % x is augmented with the consider state
                x_pred = zeros(size(x));
                % State prediction
                x_pred(1:16) = obj.nominal_model.propagate(x(1:16), u);
                x_pred(17:end) = x(17:end); 
                
                % Quaternion normalisation
                x_pred(7:10) = x_pred(7:10) / norm(x_pred(7:10)); 

                % Discretisation necessary for state covariance prediction
                I_15 = eye(15);
                obj.F_d = I_15 + obj.Ts .* obj.error_model.F(x(1:16), u);
                obj.G_d = obj.Ts .* obj.error_model.G(x(1:16));

                % State covariance prediction
                P_pred_u = obj.F_d * P(1:15,1:15) * obj.F_d' + obj.G_d * obj.Q_d * obj.G_d';  
                
                obj.F_dc = zeros(3*C, 3*C);
                obj.Q_dc = zeros(3*C,3*C);
                
                for i = 1 : 3 : 3*C
                    obj.F_dc(i:i+2, i:i+2) = eye(3);
                    obj.Q_dc(i:i+2, i:i+2) = 0.01.*eye(3);
                end
                
                P_pred_uc = obj.F_d * P(1:15, 16:15+3*C) * obj.F_dc';
                P_pred_c = obj.F_dc * P(16:15+3*C, 16:15+3*C) * obj.F_dc' + obj.Q_dc;
                P_pred = [P_pred_u P_pred_uc; P_pred_uc' P_pred_c];
            end   
            
            P_pred = (P_pred + P_pred')/2;
        end
        
        function [x_upd, P_upd, v, S] = update(obj, x, P, z, h, H, R, H_c)
            % Injects the nominal state prediction with the error state update based on
            % measurements from GNSS/PARS, IMU and compass if available, otherwise sets the
            % updated state and state covariance to the input prediction
            %
            % x Nominal state prediction
            % P Nominal state covariance prediction
            % u 6 x 1 Input from IMU 
            % z measurement vector Z x 1 
            % h predicted measurement Z x 1 
            % H measurement Jacobian / observation matrix. Z x 15 
            % R measurement noise covariance matrix Z x Z block diagonal matrix
            % H_c measurement Jacobian for the consider state Z x 3*C 
            % Make sure that the order of the measurements are given as compass, IMU and then GNSS/PARS

            if nargin == 3
                % No aiding
                x_upd = x;
                P_upd = P;
            elseif nargin == 7
                % Aiding using GNSS or beacons
                x_upd = zeros(size(x));

                % Innovation and innovation covariance
                v = z - h;  
                S = H * P * H' + R; 

                % Smallest signed angle of compass angle error
                v(1) = ssa(v(1));       
                
                % Kalman gain
                W = P * H' / S;

                % Error state estimate  
                dx = W * v;
                dq = euler2q(dx(7), dx(8), dx(9)); % Preparation for injection

                % State update with error state injection
                x_upd(1:3)   = x(1:3) + dx(1:3);
                x_upd(4:6)   = x(4:6) + dx(4:6);
                x_upd(7:10)  = quatprod(x(7:10), dq);
                x_upd(11:13) = x(11:13) + dx(10:12);
                x_upd(14:16) = x(14:16) + dx(13:15);

                % Quaternion normalisation
                x_upd(7:10) = x_upd(7:10) / norm(x_upd(7:10));
                
                G = blkdiag(eye(6), eye(3) - Smtrx(1/2.*dx(7:9)), eye(6));
                
                % State covariance update
                IWH = (eye(15) - W * H);
                P_upd = IWH * P * IWH' + W * R * W';
                P_upd = G * P_upd * G';
            else
                % Aiding using collaborative navigation, so we utilise
                % Schmidt Kalman
                x_upd = zeros(16,1);
                P_uu = P(1:15, 1:15);
                P_uc = P(1:15, 16:end);
                P_cc = P(16:end, 16:end);
                
                v = z - h;
                v(1) = ssa(v(1));
                S = H * P_uu * H' + H_c * P_uc' * H' + ...
                    H * P_uc * H_c' + H_c * P_cc * H_c' + R;
                
                W = (P_uu * H' + P_uc * H_c') / S;
                
                dx = W * v;
                dq = euler2q(dx(7), dx(8), dx(9)); 
                
                % State update with error state injection
                x_upd(1:3)   = x(1:3) + dx(1:3);
                x_upd(4:6)   = x(4:6) + dx(4:6);
                x_upd(7:10)  = quatprod(x(7:10), dq);
                x_upd(11:13) = x(11:13) + dx(10:12);
                x_upd(14:16) = x(14:16) + dx(13:15);
                
                % Quaternion normalisation
                x_upd(7:10) = x_upd(7:10) / norm(x_upd(7:10));
                
                G = blkdiag(eye(6), eye(3) - Smtrx(1/2.*dx(7:9)), eye(6));
                
                % State covariance update
                W_aug = [W; zeros(9,13)];
                IWH_aug= eye(15+size(H_c,2)) - W_aug*[H H_c];
                P_upd = IWH_aug*P*IWH_aug' + W_aug*R*W_aug';
                P_upd(1:15,1:15) = G *  P_upd(1:15,1:15) * G';                
            end 
        end
    end
end


classdef Agent < handle
    % Agent class
    
    % ID
    properties
        id
        name
    end
    
    % Simulation parameters
    properties 
        Ts % Discrete time step length
        n  % Intitial time steps to omit for filter consistency analysis
        N  % Number of simulation time steps
        pregen % Flag indicating whether the true state trajectory has already been created 
    end
    
    % State parameters
    properties 
        x_t  % True state vector 16 x N
        x    % Nominal state vector 16 x N
        P    % State covariance 15 x 15 x N 
        P_uc % Current update-consider cross-covariance 
        a_n  % Acceleration in NED-frame Nx3
        w_n  % Angular velocity in BODY-frame Nx3
        u    % IMU input 6xN
    end
    
    % Kinematics
    properties
        nominal_model
        error_model
    end
    
    %  Filter and sensors
    properties 
        seskf % (Schmidt) ESKF
        GNSS_receiver 
        PARS_receiver 
        IMU           
        Compass   
        GNSS_denied % Flag indicating whether agent is operating in GNSS denied environment 
        connected_agents % Array of connected agents
        C % num of connected agents
    end
    
    % Filter consistency
    properties
        v % Cell array length (N-n)*Ts Innovation
        S % Cell array length (N-n)*Ts Innovation covariance
        fcidx 
    end
    
    methods
        function obj = Agent(id, name)
            %AGENT Initialises an empty agent instance with the given ID
            %and (potential) name. 
            % id Agent ID
            % name Agent name, can be omitted, at which point agent name is
            % Agent + ID
            obj.id = id;

            if nargin == 1
                obj.name = strcat("Agent ", id);
            else
                obj.name = name;
            end
        end
        
        function set_simulation_parameters(obj, Ts, n, N)
            %SET_SIMULATION_PARAMETERS Set the agent's simulation parameters
            % Ts Time step length
            % n  Time steps to be omitted for filter consistency
            % N  Total number of time steps
            obj.Ts = Ts;
            obj.n = n;
            obj.N = N;
            obj.pregen = 0;
        end
        
        function init_state_parameters(obj, x0, P0)
            %INIT_STATE_PARAMETERS
            % Inititalise state with initial conditions
            % x0 16x1 Initial state vector w/ normalised quaternion att.
            % P0 15x15 Initial state covariance
            can_init = ~isempty(obj.N) && ~isempty(obj.n) && ~isempty(obj.Ts);
            
            if can_init == 1
               obj.x = zeros(16, obj.N);
               obj.x_t = zeros(16, obj.N);
               obj.P = zeros(15, 15, obj.N);
               
               % At rest initially
               obj.a_n = zeros(obj.N, 3);
               obj.w_n = zeros(obj.N, 3);
               obj.u = zeros(6,obj.N);

               obj.x_t(:, 1) = x0;
               obj.x(1:3,1) = x0(1:3);
               obj.x(7:10,1) = x0(7:10);
       
               if nargin == 3
                  obj.P(:, :, 1) = P0; 
               end
               
               obj.v = cell(1, floor((obj.N - obj.n)*obj.Ts));
               obj.S = cell(1, floor((obj.N - obj.n)*obj.Ts));
               obj.fcidx = 1;   
            else
                error("Simulation parameters not initialised");
            end
        end
        
        function init_kinematic_models(obj)
            %INIT_KINEMATIC_MODELS 
            % Initialises the kinematic models, if the simulation
            % parameters have been set
            if ~isempty(obj.Ts)
                obj.nominal_model = Agent3DNominal(obj.Ts);
                obj.error_model = Agent3DError(obj.Ts);
            else
                error("Cannot initialise kinematics, Ts not defined");
            end
        end
        
        function init_filter(obj, Q)
            %INIT_FILTER
            % Initialises the filter with the given process noise
            % covariance matrix. 
            % Q_d 12 x 12 diagonal matrix. vel, quat, b_acc, b_gyro
            can_init = ~(isempty(obj.Ts) || isempty(obj.nominal_model) || ...
                       isempty(obj.error_model));
                 
            if can_init == 1
                num_rows = size(Q,1);
                num_cols = size(Q,2);
                
                if ~((num_rows == 12) && (num_cols == 12))
                    error("Dimensions of process noise covariance incorrect, must be 12x12");
                else
                    obj.seskf = SESKF(obj.Ts, obj.nominal_model, obj.error_model, Q);
                end 
            else 
                error("Cannot initialise filter, sim. parameters and/or models not initialised");
            end
        end
        
        function init_sensors(obj)
            %INIT_SENSORS
            % Initialises the various sensors of the agent with
            % predetermined noise parameters.
            if ~isempty(obj.Ts)
                obj.GNSS_receiver = GNSS_Sensor([0 0 0]', [1.5 1.5 10]');
                obj.IMU = IMU_Sensor(obj.Ts);
                obj.Compass = Compass_Sensor(0,1);
                obj.PARS_receiver = PARS_Sensor(0,0,0,7,0.3*deg2rad(1),0.3*deg2rad(1));
            else
                error("Cannot initialise sensors, time step not initialised");
            end
        end
        
        function set_GNSS_denied(obj, status)
            %SET_GNSS_DENIED
            % Status boolean sets GNSS availablity
            % 0 GNSS available
            % 1 Not available, navigating collaboratively
           obj.GNSS_denied = status; 
           
           fprintf(sprintf("GNSS denied status for %s = %d\n", obj.name, status));
        end
        
        function set_connected_agents(obj, agents)
           %SET_CONNECTED_AGENTS
           % Takes in an row vector of remote agents and sets the agent's
           % connected_agents variable to it, sets  the
           % variable C to the number of elements and initialises the
           % cross-covariance of the SKF
           % agents 1xC vector of Agent objects
           obj.connected_agents = agents;
           
           obj.C = size(agents,2);
           
           obj.P_uc = zeros(15,3*obj.C);
        end
        
        function set_true_state(obj, x_t, a, w)
            %SET_TRUE_STATE
            % Sets the true state of the agent and subsequently activated
            % the pregen flag. In this case the true trajectory won't be
            % generated during the simulation
            % x_t 16 x N matrix with the agent's 16-dim state for the time steps. 
            % a Nx3 matrix with the agent's generated NED acceleration 
            % w Nx3 matrix with the agent's generated NED angular velocity
            if size(x_t, 1) == 16
                obj.x_t = x_t; 
                obj.pregen = 1;
                obj.a_n = a;
                obj.w_n = w;
            else
                error("Cannot set true trajectory, must be a 16 x N matrix");
            end
        end
        
        function set_measurement_noise(obj, u, v_gnss, v_pars, v_comp)
           %SET_MEASUREMENT_NOISE
           % For pregenerated cases, set noise vectors of the sensors in advance
           % u 6 x N Noisy IMU measurements
           % v_gnss 3 x N*Ts GNSS noise 
           % v_gnss 3 x N*Ts PARS noise
           % v_comp 1 x N*Ts Compass noise
           obj.IMU.set_data(u);
           obj.GNSS_receiver.set_data(v_gnss);
           obj.PARS_receiver.set_data(v_pars);
           obj.Compass.set_data(v_comp);
        end
        
        function propagate_imu_input(obj, k)
            %PROPAGATE_IMU_INPUT
            % Propagates the IMU input
            % k Current time step from k = 2
      
            % Get the acceleration and angular velocity measurement from
            % the IMU
            obj.u(:,k) = obj.IMU.get_signal_att(obj.a_n(k,:), obj.w_n(k,:), obj.x_t(7:10, k));
        end
        
        function update(obj, k)  
            %UPDATE
            % Uses current estimate and collects measurements (if k*obj.Ts
            % = f_aid) and updates the state estimate
            % k Current time step from k = 2
            u_k = obj.u(:,k-1);
            
            if (obj.GNSS_denied ~= 0)
                x_aug = zeros(16+3*obj.C,1);
                x_aug(1:16) = obj.x(:,k-1);
                P_aug = zeros(15+3*obj.C,15+3*obj.C);
                P_aug(1:15,1:15) = obj.P(:,:,k-1);
                P_aug(1:15,16:15+3*obj.C) = obj.P_uc; % This assumes nonchanging set of connected agents
                agent_idx = 1;
                for i = 1 : 3 : 3*obj.C
                    x_aug(16+i:16+i+2) = obj.connected_agents(agent_idx).x(1:3, k-1);
                    P_aug(15+i:15+i+2, 15+i:15+i+2) = obj.connected_agents(agent_idx).P(1:3,1:3,k-1);
                    agent_idx = agent_idx + 1;
                end

                % State prediction
                [x_pred, P_pred] = obj.seskf.predict(x_aug, P_aug, u_k, obj.C);
            else
                [x_pred, P_pred] = obj.seskf.predict(obj.x(:,k-1), obj.P(:,:,k-1), u_k);
            end
            
            if mod(k * obj.Ts, 1) == 0  
                % If mod (k * agent.Ts, 1) == 0 then aiding measurements
                
                % Get the IMU measurement model
                [z_imu, h_imu, H_imu, R_imu] = obj.IMU.get_measurement(x_pred(7:10), obj.u(:,k));

              
                % Get the true and nominal heading angle for the compass
                [z_comp, h_comp, H_comp, R_comp] = obj.Compass.get_heading_measurement(x_pred(7:10), obj.x_t(7:10,k));
                
                if (obj.GNSS_denied == 0)
                    % GNSS measurement
                    [z_gnss, h_gnss, H_gnss, R_gnss] = obj.GNSS_receiver.get_position_measurement(obj.x_t(1:3, k)); 

                    % Construct the full measurement model
                    z = [z_comp; z_imu; z_gnss];
                    h = [h_comp; h_imu; h_gnss(x_pred(1:3))];
                    H = [H_comp; H_imu; H_gnss];
                    R = blkdiag(R_comp,R_imu, R_gnss);
                    
                    [obj.x(:,k), obj.P(:,:,k), vk, Sk] = obj.seskf.update(x_pred(1:16),P_pred(1:15,1:15),z,h,H,R);        
                else
                    % Assume placement of PARS receiver coincide with GNSS
                    % receiver
                    RNB = eye(3);
                    p0 = obj.x_t(1:3,k);
                    zp = zeros(3*obj.C, 1);
                    hp = zeros(3*obj.C, 1);
                    Hp = zeros(3*obj.C, 15);
                    Hp_c = zeros(3*obj.C, 3*obj.C);
                    Rp = zeros(3*obj.C, 3*obj.C);
                    
                    for a = 1 : obj.C
                       p1 = obj.connected_agents(a).x_t(1:3,k);
                       sidx = 1 + obj.C*(a-1);
                       eidx = 3 + obj.C*(a-1);
                       [zp(sidx:eidx), hpa, Hp(sidx:eidx,:), Rp(sidx:eidx,sidx:eidx), Hp_c(sidx:eidx, sidx:eidx)] = obj.PARS_receiver.get_position_measurement(p0, p1, RNB, "schmidt");
                       hp(sidx:eidx) = hpa(x_pred(1:3), x_pred(16+sidx:16+eidx));
                    end
                    
                    obj.PARS_receiver.increment_pregen_idx();
                    
                    z_PARS = zp;
                    h_PARS = hp;
                    H_PARS = Hp;
                    R_PARS = Rp;
                    
                    % Construct the full measurement model
                    z = [z_comp; z_imu; z_PARS];
                    h = [h_comp; h_imu; h_PARS];
                    H = [H_comp; H_imu; H_PARS];
                    H_c = [zeros(4,3*obj.C); Hp_c];
                    R = blkdiag(R_comp,R_imu, R_PARS);
                    
                    
                    % State update
                    [x_upd, P_aug, vk, Sk] = obj.seskf.update(x_pred,P_pred,z,h,H,R, H_c);
                    obj.x(:,k) = x_upd;
                    obj.P(:,:,k) = P_aug(1:15,1:15);
                    obj.P_uc = P_aug(1:15,16:end);
                end
                
                if k > obj.n
                    obj.v{obj.fcidx} = vk;
                    obj.S{obj.fcidx} = Sk;
                    obj.fcidx = obj.fcidx + 1;
                end
            else
                % Otherwise no aiding, so prediction becomes estimate
                [obj.x(:,k), obj.P(:,:,k)] = obj.seskf.update(x_pred(1:16),P_pred(1:15,1:15));
            end
        end
    end
end


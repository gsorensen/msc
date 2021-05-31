classdef Agent3DError
    %AGENT3DError
    %   Implementation of the discrete-time error-state state-space model 
    %   defined in section 4.3.2 in the thesis
    properties
       Ts 
       F
       G
    end
    
    methods
        function obj = Agent3DError(Ts)
           obj.Ts = Ts; 
           
           R   = @(x) Rquat(x(7:10));
           S_a = @(x, u) Smtrx(u(1:3) - x(11:13));
           S_m = @(x, u) Smtrx(u(4:6) - x(14:16));
           
           I_3 = eye(3);
           O_3 = zeros(3);
           
           obj.F = @(x, u) [O_3 I_3 O_3 O_3 O_3
                            O_3 O_3 -R(x)*S_a(x, u) -R(x) O_3
                            O_3 O_3 -S_m(x, u) O_3 -I_3
                            O_3 O_3 O_3 O_3 O_3
                            O_3 O_3 O_3 O_3 O_3];
                        
           obj.G = @(x) [O_3 O_3 O_3 O_3
                         -R(x) O_3 O_3 O_3
                          O_3 -I_3 O_3 O_3
                          O_3 O_3 I_3 O_3
                          O_3 O_3 O_3 I_3];                       
        end    
    end
end
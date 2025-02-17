classdef ATV_Dynamics_rungakutta <handle
    properties
           %parameters
        ell
        r
        v

        %sim paramters
        phi_throw

        previous_z
        position
        states
        states_dot
        Ts

        %saturation
        phi_max_speed
        phi_min_speed
    end
    methods 
        function self = ATV_Dynamics_rungakutta(Param)
            self.ell = Param.ell;                       %wheel base length
            self.r   = Param.r;                         %distance between center of rotation and connectiuon point of linear actuator onto wheel
        %intialize states and positons
            z_intial = self.r*sin(Param.phi_0);
            self.previous_z = z_intial;                            %get current postion of linear actuator
            self.position = [Param.x_0; Param.y_0];
            self.states   = [Param.v ; Param.chi_0 ; Param.phi_0]; %v is velocity of cart, chi is carts current heading, and phi is the wheel base angle or steering angle
            self.states_dot = [0;0;0];
            self.Ts       = Param.t_s;
        %saturation parametrs
            self.phi_max_speed  = Param.phi_max_speed;
            self.phi_min_speed  = -Param.phi_max_speed;
            self.v              = Param.v;
        end

        function self= update(self, input,path_completed)
            self.rk4_step(input); 
            if path_completed == 1
                self.v = 0;
            end

        end
        
        function [position_dot, states_dot] = dynamics(self, position, states, input)
            %unpack
            x   = position(1);
            y   = position(2);
            v   = self.v;%states(1);
            chi = states(2);
            phi = states(3);
            
            %grab pwm input and convert that to a velocity and use numerical integration to calculate where linear actuator position is
            pwm             = 4*input(2) +1520;
            z_dot_a         = (self.phi_max_speed/(1920-1520))*pwm - (self.phi_max_speed/(1920-1520))*1520;
            z               = sin(phi) * self.r;
            self.previous_z =z; 

            e_in    = input(1);     %voltage input to command velocity 
            
            %parameters
            ell = self.ell;
            r   = self.r;

            %dynamic equations
            x_dot   = v*sin(chi);
            y_dot   = v*cos(chi);
            phi_dot = z_dot_a/(r * cos(phi));
            chi_dot = v/ell * tan(phi);
            v_dot = 0;

            %pack
            states_dot   = [v_dot; chi_dot; phi_dot];
            self.states_dot = states_dot;
            position_dot = [x_dot; y_dot];

        end

        function rk4_step(self, input)
            % Integrate ODE using Runge-Kutta RK4 algorithm
            [position_dot_1,states_dot_1] = self.dynamics(self.position,self.states,                          input);
            [position_dot_2,states_dot_2] = self.dynamics(self.position + self.Ts/2*position_dot_1, self.states + self.Ts/2*states_dot_1, input);
            [position_dot_3,states_dot_3] = self.dynamics(self.position + self.Ts/2*position_dot_2,self.states + self.Ts/2*states_dot_2, input);
            [position_dot_4,states_dot_4] = self.dynamics(self.position + self.Ts* position_dot_3,self.states + self.Ts  *states_dot_3, input);
            self.states   =   self.states + self.Ts/6 * (states_dot_1 + 2*states_dot_2 + 2*states_dot_3 + states_dot_4);
            self.position = self.position + self.Ts/6 * (position_dot_1 + 2*position_dot_2 + 2*position_dot_3 + position_dot_4);
        end


    end
end
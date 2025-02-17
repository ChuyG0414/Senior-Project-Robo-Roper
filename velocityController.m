classdef velocityController < handle
    properties

        %saturation limits
        e_in_max
        e_in_min
        % Control States
        e_integral

        % Values form previous time step
        previous_e

        % Time Step
        Ts

        % Gains
        k_p
        k_i
        k_d
    end

    methods
        function self = velocityController(P)
            self.e_in_max      = P.e_in_max;
            self.e_in_min      = P.e_in_min;
            self.Ts         = P.t_s;
            self.previous_e = 0;
            self.e_integral = 0;

            % Control gains
            self.calculate_gains(P.v_t_r, P.v_zeta, P.v_t_r_integrator)
           
        end

        function u_sat = update(self,states, states_dot, reference)
            v = states(1);
            y = states(1);
            y_dot = states_dot(1);
            r = reference;

            %error
            e = r - y;

             % Integrate Error
            self.e_integral = self.e_integral + self.Ts/2*(e + self.previous_e);
            self.previous_e = e;

            % PID Control
            u = self.k_p*e + self.k_i*self.e_integral ;%- self.k_d*y_dot;
            
            u_sat = self.velocity_saturate(u,self.e_in_min, self.e_in_max);
            
            %anti integrator wind up
            self.e_integral = self.e_integral + self.Ts/self.k_i*(u_sat - u);


        end

        function calculate_gains(self,v_t_r, v_zeta, v_t_r_integrator)
             % Natural Frequencies
            omega_n = pi/(2*v_t_r*sqrt(1-v_zeta^2)); % Eq 8.5 form book (for complex poles). Approximately 2.2/t_r
            integrator_pole = log(1/9)/v_t_r_integrator; % Derived by hand (for real poles). Approximately 2.2/t_r
            % Desired Charictoristic Equation
            alpha_0 = -omega_n^2*integrator_pole;
            alpha_1 = omega_n^2 - 2*v_zeta*omega_n*integrator_pole;
            alpha_2 = 2*v_zeta*omega_n - integrator_pole;
            
            %coefficents from transfer function 
            a_0 = 0;
            a_1 = .4438;
            b_0 = .3778;            %derrived from least squares analysis of velocity

            % PID Gains
            k_p = (alpha_1 - a_0)/b_0;
            k_i = (alpha_0)/b_0;
            k_d = (alpha_2 - a_1)/b_0;

            self.k_p = k_p;
            self.k_i = k_i;
            self.k_d = k_d;
        end

        function u_sat = velocity_saturate(self, u, u_min, u_max)
            if u > u_max
                u_sat = u_max;
            elseif u < u_min
                u_sat = u_min;
            else
                u_sat = u;
            end
        end

    end
end
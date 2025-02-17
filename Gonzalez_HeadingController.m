%heading contoller 
classdef Gonzalez_HeadingController < handle
    properties
        kp
        kd
        ki
        ell
        
        Ts
        upper_limit
        lower_limit
        previous_e
        e_integral
        sigma
         
        theta_previous
        theta_dot_prev
        theta_dot

        max_phi
        min_phi
        velocity
    end
    methods
        function self = Gonzalez_HeadingController(Param)
            self.theta_previous = Param.chi_0;
            self.ell = Param.ell;
            self.Ts = Param.t_s;
            self.upper_limit = Param.theta_upper_limit;
            self.lower_limit = Param.theta_lower_limit;
            self.previous_e = 0;
            self.e_integral = 0;
            self.sigma = Param.sigma_heading;
            self.theta_dot_prev = 0;
            self.theta_dot = 0;
            self.ki = Param.ki_theta;
            self.velocity = Param.v;
            self.max_phi = Param.steering_max_angle;
            if isnan(Param.kp_theta)
                self.calculate_gains(Param.t_r_theta,Param.zeta_theta);
            else
                self.kp = Param.kp_theta;
                self.kd = Param.kd_theta;
            end


  

        end
        function phi_ref = update(self,states,reference)
            theta = states(2);
            theta_dot = states(5);
            self.theta_previous = theta;
            self.theta_dot = self.differentiate_theta(theta,self.theta_previous);
            r = reference;
            %error
            error = r-theta;

            %integrate error
            self.e_integral = self.e_integral + self.Ts/2*(error + self.previous_e);
            self.previous_e = error;

            %PID
            u = self.kp*error + self.ki*self.e_integral - self.kd*theta_dot;%(error - self.previous_e)/self.Ts
            u_sat = self.headingSaturate(u,self.upper_limit,self.lower_limit);

            % Saturation based anti-windup
%             self.e_integral = self.e_integral + self.Ts/self.ki*(u_sat-u);
         

            
            phi_ref = self.map(u_sat, self.lower_limit, self.upper_limit, -self.max_phi, self.max_phi);
        end
        function calculate_gains(self,t_r_theta,zeta_theta)

            omega_theta = pi/(2*t_r_theta*sqrt(1-zeta_theta^2));
            alpha_0 = omega_theta^2;
            alpha_1 = 2*zeta_theta*omega_theta;
            
            self.kd = self.ki/alpha_0 - self.ell/self.velocity;
            self.kp = (2+ self.kd)*alpha_1/self.velocity;

        end

        function theta_dot = differentiate_theta(self,theta,theta_previous)
            theta_dot = ((2*self.sigma-self.Ts)/(2*self.sigma+self.Ts))*self.theta_dot + (2/(2*self.sigma + self.Ts)) * (theta - theta_previous);
            self.theta_dot_prev = theta_dot;
        end

        function out = headingSaturate(self,in,upper_limit,lower_limit)
            if in >upper_limit
                out = upper_limit;
            elseif in < lower_limit
                out = lower_limit;
            else
                out = in;
            end
        end
        function y = map(self,x, in_min, in_max, out_min, out_max)
    % This function maps the input x from the range [in_min, in_max] 
    % to the range [out_min, out_max].
    
    y = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        end


    end
end







%linear actuator controller 
classdef Gonzalez_SteeringAngleController < handle
    properties
         kp
         kd
         ki
         t_r_z
         t_r_z_integrator
         zeta_z
         r
         Ts
         upper_limit
         lower_limit
         previous_e
         e_integral
         sigma
         
         z_previous
         z_dot_prev
         z_dot
 end
 methods
 %------constructor----------------------
    function self = Gonzalez_SteeringAngleController(Param)
        self.t_r_z = Param.t_r_z;
        self.t_r_z_integrator = Param.t_r_z_integrator;
        self.zeta_z = Param.zeta_z;
        self.upper_limit = Param.upper_limit;
        self.lower_limit = Param.lower_limit;
        self.Ts = Param.t_s;
        self.previous_e = 0;
        self.e_integral = 0;
        self.sigma = Param.sigma_LA;
        self.r = Param.r;
        self.z_previous = self.r*tan(Param.phi_initial);
        self.z_dot_prev = 0;
        self.z_dot = 0;
        self.ki = Param.ki_phi;
        self.kp = Param.kp_phi;
        self.kd = Param.kd_phi;
        %self.calculate_gains()

    end
    function u_sat = update(self, reference, states)
        %calculate my input steps needed, calculate error, integrate error , differintiate error 
        %calculate gains, and implement PID controller
        
        %error
        phi_ref = reference;
        z_r = self.r*sin(phi_ref);
        error = z_r - states(1);
        self.z_dot = self.differentiate_z(states(1),self.z_previous);%states(2);%

        %integrate error
        self.e_integral = self.e_integral + self.Ts/2*(error + self.previous_e);
        self.previous_e = error;

        u = self.kp*error + self.ki*self.e_integral - self.kd*self.z_dot;%(error - self.previous_e)/self.Ts
        u_sat = self.saturate(u);
        if self.ki == 0
             self.e_integral;
        else
            self.e_integral = self.e_integral + self.Ts/(self.ki)*(u_sat-u);
      
        end
        self.z_previous = states(1);
        

    end
    function calculate_gains(self)
                %calculate omega and integrtor pole
        wn_z = pi/(2*self.t_r_z*sqrt(1-self.zeta_z^2));
        integrator_pole = log(1/9)/self.t_r_z_integrator;
        % Desired Charictoristic Equation
        alpha_0 = wn_z^2;
        alpha_1 = 2*self.zeta_z*wn_z;
        alpha_2 = 1;

        self.kd = self.ki/alpha_0 - 1;              %kp and kd are inversely proporitonal, kind of tricky to tune 
        self.kp = (self.kd + 1)*(alpha_1);
        
    end     

     function I = integrate_e(self, error)
         self.integrator = self.integrator...
         + (self.Ts / 2) * (error + self.error_d1);
         self.error_d1 = error;
         I = self.integrator;
     end
 
    function zdot = differentiate_z(self,z,z_previous)
         zdot = ((2*self.sigma-self.Ts)/(2*self.sigma+self.Ts))*self.z_dot + (2/(2*self.sigma + self.Ts)) * (z - z_previous);
         self.z_dot_prev = zdot;
    end
    function out = saturate(self, in)   %redefine the saturate function
         if in > self.upper_limit
            out = 100;%self.limit * sign(u);
         elseif in < self.lower_limit
            out = -100;
         else
         out = in;
         end
    end
    
    end
end

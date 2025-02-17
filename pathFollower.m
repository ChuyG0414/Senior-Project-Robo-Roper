classdef pathFollower < handle
    properties
    % Gains
        chi_inf
        k_path
        k_orbit

        % Time step
        Ts 
    end
    methods
        
        function self = pathFollower(Param) 
        %need to intitalize my constructor not sure what information to have user first input
        %im thinking initial conditions
        %self object is being intialized internally so I have created an object within my class
        %that i can call and use methods with and change parameters
            self.chi_inf = Param.chi_inf;
            self.k_path = Param.k_path;
            self.k_orbit = Param.k_orbit;
  
        end
        function  [v_r, chi_r] = update(self, position, y_manager, chi_0)
            %unpack
            p_east  = position(1);
            p_north = position(2);
            p_down  = 0;
            p       = [p_north, p_east, p_down];
            flag    = y_manager(1);
            v_r     = y_manager(2);
            r       = y_manager(3:5);
            q       = y_manager(6:8);
            c       = y_manager(9);
            rho     = y_manager(10);
            lambda  = y_manager(11);
            chi     = chi_0;
            % Path Follower Algorithm ------------------------------------
            switch flag
                case 1
                    chi_r = self.straightLineFollowing(r, q, p, chi);
                case 0
                    chi_r = self.circleFollowing(c, rho, lambda, p, chi);
            end

        end
        function chi_command = straightLineFollowing(self, r, q, p, chi)
        %this function uses the straightLinefollowing, 
        %r is the vector from orgin (home point) to line we want to follow
        %q is a unit vector pointing in the direction we want to follow, defined as q = (qn, qe, qd)
        %p is a vector pointing from home to our current position
        %chi current heading of cart
        %chi_q = heading of path we want
        %chi_command = what we command chi to be
        %path manager hands me the inputs to this function  

            chi_q =atan2(q(2),q(1));
            

        %ask jacob to make sense of why we do the following
            while chi_q - chi < -pi
                chi_q = chi_q + 2*pi;
            end
            while chi_q - chi > pi
                chi_q = chi_q - 2*pi;
            end

            T_inertial_to_path = [cos(chi_q) sin(chi_q) 0 ; 
                              -sin(chi_q) cos(chi_q) 0; 
                                0             0      1];

            %calculate error in inertial frame from r(vector from orgin to 
            %line we'd like to be at) - p (positional vector of where we currently are)
            error_path = T_inertial_to_path * (p - r);
            e_py = error_path(2);
            e_p_y = -sin(chi_q)*(p(1)-r(1))+cos(chi_q)*(p(2)-r(2));
            chi_command = chi_q - self.chi_inf * 2/pi * atan(self.k_path * e_p_y);

        end
        function chi_command = circleFollowing(self, c,rho, lambda, p, chi)
            
            d = sqrt( (p(1)-c(1))^2 + (p(2) - c(2))^2 );
            phi_d = atan2( p(2) - c(2), p(1)-c(1) );
            while phi_d - chi <-pi 
                phi_d = phi_d + 2*pi;
            end
            while phi_d - chi > pi
                phi_d = phi_d - 2*pi;
            end
            chi_deg = phi_d + lambda*pi/2;
            chi_command = chi_deg + lambda*atan(self.k_orbit * ( (d - rho)/rho ) );
        end
    end
end
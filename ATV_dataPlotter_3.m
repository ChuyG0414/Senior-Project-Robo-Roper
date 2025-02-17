classdef ATV_dataPlotter_3 < handle
    properties
        %data historys
        time_history
        chi_ref_history
        v_ref_history
        phi_ref_history
        v_history
        phi_history
        chi_history
        e_in_history
        LA_effort_history
        
        x_history
        y_history
        
        %waypoint data
        x_waypoints
        y_waypoints
        
        %figure handles
        chi_ref_handle
        v_ref_handle
        phi_ref_handle
        v_handle
        phi_handle
        chi_handle
        e_in_handle
        LA_effort_handle
        position_handle
        waypoint_handle

        index
        %throw away 
    end
    methods
        %--constructor--------------------------
        function self = ATV_dataPlotter_3(P)
            %intilize index
            self.index = 1;
            % Instantiate lists to hold the time and data histories
            self.time_history    = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.chi_ref_history       = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.phi_history  = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.phi_ref_history  = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.v_ref_history  = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.v_history  = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);

            self.chi_history  = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.e_in_history = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.LA_effort_history = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.x_history    = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.x_history(1) = P.x_0;
            self.y_history    = NaN*ones(1, (P.t_end - P.t_start) / P.t_plot);
            self.y_history(1) = P.y_0;
            %unpack waypoints we want to follow
            self.x_waypoints  = P.W(1,:);
            self.y_waypoints  = P.W(2,:);

            %intialize handles

            %states figure
            figure(1), clf
            subplot(311)
            hold on 
            self.v_handle  = plot(self.time_history,self.v_history,'b');
            self.v_ref_handle  = plot(self.time_history,self.v_ref_history,'g');
            title('Velocity')
            ylabel('m/s')
            subplot(312)
            hold on 
            self.chi_ref_handle   = plot(self.time_history, self.chi_ref_history, 'g');          %make sure we convert from rads to deg only in the data plotting 
            self.chi_handle = plot(self.time_history, self.chi_history, 'b');
            ylabel("Degrees(\circ)");
            title("Heading");
            
            subplot(313)
            hold on 
            self.phi_ref_handle = plot(self.time_history, self.phi_ref_history, 'g');            
            self.phi_handle = plot(self.time_history, self.phi_history, 'b');
            ylabel("Degrees(\circ)");
            legend('Phi-Runga')
            title("steering angle");

            %input figure
            figure(2)
            subplot(211)
            hold on
            self.e_in_handle = plot(self.time_history, self.e_in_history,'b');
            ylim([0 5])
            ylabel("voltage (v)");
            title("Voltage to Cart Speed Controller");
            subplot(212)
            hold on
            self.LA_effort_handle = plot(self.time_history, self.LA_effort_history,'b');
            title("effort of LA");
            ylabel("effort");
            ylim([-100,100]);

            %position figure
            figure(3)
            self.position_handle = plot(self.x_history, self.y_history, 'b', self.x_waypoints,self.y_waypoints,'g--o');
            title("Position");
            ylabel("(m)");
            xlabel("(m)");
            axis([-20 20 -20 60]);
            

            
            
        end

        function self = update(self, time, reference, position, vehicle_states, control)
            % Update the time history of all plot variables

            %unpack reference values
            v_ref   = reference(1);
            chi_ref = reference(2);
            phi_ref = reference(3);

            %unpack state values
            v   = vehicle_states(1);
            chi = vehicle_states(2);
            phi = vehicle_states(3);

            %unpack positon
            x = position(1);
            y = position(2);

            %unpack control inputs 
            e_in      = control(1);
            LA_effort = control(2);

            %update time history
            self.time_history(self.index) = time;
            %reference history
            self.v_ref_history(self.index)       = v_ref;
            self.chi_ref_history(self.index) = rad2deg(chi_ref);                     %reference heading
            self.phi_ref_history(self.index) = rad2deg(phi_ref);

            %state histroy
            self.v_history(self.index)   = v;
            self.chi_history(self.index)  = rad2deg(chi);             %cart heading 
            self.phi_history(self.index)  = rad2deg(phi);             %cart steering angle

            %positon history
            self.x_history(self.index+1) = x;                               %x position of cart
            self.y_history(self.index+1) = y;                               %y position of cart
            
            %control input history
            self.e_in_history(self.index)     = e_in;             
            self.LA_effort_history(self.index) = LA_effort; 

            %step index
            self.index = self.index + 1;

            % Update the plots with associated histories
            %states figure
            set(self.v_ref_handle, 'Xdata', self.time_history, 'Ydata', self.v_ref_history)
            set(self.v_handle, 'Xdata', self.time_history, 'Ydata', self.v_history)

            set(self.chi_ref_handle, 'Xdata', self.time_history, 'Ydata', self.chi_ref_history)
            set(self.chi_handle, 'Xdata', self.time_history, 'Ydata', self.chi_history)

            set(self.phi_ref_handle, 'Xdata', self.time_history, 'Ydata', self.phi_ref_history)
            set(self.phi_handle, 'Xdata', self.time_history, 'Ydata', self.phi_history)

            %input figure
            set(self.e_in_handle, 'Xdata', self.time_history, 'Ydata', self.e_in_history)
            set(self.LA_effort_handle, 'Xdata', self.time_history, 'Ydata', self.LA_effort_history)            
            
            %position figure
            set(self.position_handle(1), 'Xdata', self.x_history, 'Ydata', self.y_history)   
            set(self.position_handle(2), 'Xdata',self.x_waypoints, 'Ydata', self.y_waypoints)
            drawnow

        end
    end
end

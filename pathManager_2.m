classdef pathManager_2 < handle
%takes list of way points to determine Y man
%Y man = [flag, V, r, q, c, rho, lambda]
%the yman is fed into the path following class
    properties
        V
        index               %point index
        way_points
        waypoint_path
        heading_threshold_for_deceleration
        v_max
    end

    methods

        function self = pathManager_2(Param)
            self.v_max      = Param.v_max;
            self.V          = Param.v_max;
            self.index      = 2;
            self.way_points = [0 1 0;1 0 0; 0 0 -1] * Param.W ;
            self.heading_threshold_for_deceleration = Param.heading_threshold_for_deceleration;
        end
        function [y_manager, path_completed] = update(self,state)
        %updates current position 
        %state vectors uses east north up cordinate system
            pn = state(2);
            pe = state(1);
            pd = 0;%state(3);  

            p = [pn; pe; pd];

            [y_manager, path_completed] = self.followWaypoints(self.way_points, p);
        end

        function [y_manager, path_completed] = followWaypoints(self,W, p)
            if ~isequal(self.way_points, W)
                self.waypoint_path = W;
                self.index = 2;
            end
            N = length(W);                      %number of waypoints
            i = self.index;                     %indexing through way points
            %check to see if we have gone through all the way points.
            if i == (N+1)
                path_completed = true;
                r = zeros(3,1);
                q = zeros(3,1);
                c = 0;
                rho = 0;
                lambda = 0;
                self.V = 0;
                y_manager  = [1 ; self.V ; r ; q ; c ; rho; lambda];
           
                %self.index = 2;
            elseif i == N
          
                path_completed = false;

                % Base point for path is start of path
                r = W(:,i-1);

                % Direction of Path
                q_im1 = (W(:,i  )-W(:,i-1))/norm(W(:,i  )-W(:,i-1));
                
                %check if we've passed point
                if (p-W(:,i))'*q_im1 >= 0
                    self.index = self.index + 1;
                end
                self.V = self.v_max/2;
                c = 0;
                rho = 0;
                lambda = 0;
                y_manager  = [1 ; self.V ; r ; q_im1 ; c ; rho; lambda];
            else
                path_completed = false;

                % Base point for path
                r = W(:,i-1);
    
                % Direction of path
                q_im1 = (W(:,i  )-W(:,i-1))/norm(W(:,i  )-W(:,i-1));

                % Direction of next path
                qi   = (W(:,i+1)-W(:,i  ))/norm(W(:,i+1)-W(:,i  ));

                % Normal vector for plane spliting paths
                ni   = (q_im1+qi)/norm(q_im1+qi);

                % Check if we've pass the waypoint
                if (p-W(:,i))'*ni >= 0
                    self.index = self.index + 1;
                end
                
                if abs(atan2(qi(2),qi(1))) - abs(atan2(q_im1(2),q_im1(1)) ) > self.heading_threshold_for_deceleration
                    self.V = self.v_max/2;
                else
                    self.V = self.v_max;
                end
                c = 0;
                rho = 0;
                lambda = 0;
                y_manager  = [1 ; self.V ; r ; q_im1 ; c ; rho; lambda];
            end
            %compute Y manager, r = previous way point 
%             r = W(:,self.index - 1);              %vector pointing to first way point
%             q_im1= ( W(:,self.index) - W(:,self.index - 1) )/norm( W(:,self.index) - W(:,self.index - 1) );
%             q_i = ( W(:,self.index + 1) - W(:,self.index) )/norm( W(:,self.index + 1) - W(self.index) );
%             n_i = (q_im1 + q_i)/norm(q_im1 + q_i);
%             c = 0;
%             rho = 0;
%             lambda = 0;
% 
%             %check to see if we have passed way point
%             if (W(:,self.index)-p)'*n_i <= 0
%                     self.index = self.index + 1;
%             end
%    
%             y_manager      = [1 ; self.V ; r ; q_im1 ; c ; rho; lambda];
%             path_completed = false;
            
        end
    end
end
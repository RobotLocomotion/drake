classdef ContactWrenchVisualizer < BotVisualizer
  % This is a hack to visualize the contact wrenches and the centroidal momenta, together
  % with the robot
  properties(SetAccess = protected)
    t_knot
    wrench_sol
    wrench_lcmgl = LCMGLClient('wrench');
    atlas
  end
  
  properties(Access = protected)
    num_contact_bodies
    force_scaler
    torque_scaler
  end
  
  methods
    function obj = ContactWrenchVisualizer(manip,t_knot,wrench_sol,use_collision_geometry)
      if nargin <4 
        use_collision_geometry = false; 
      end
      obj = obj@BotVisualizer(manip,use_collision_geometry);
      if(~isnumeric(t_knot))
        error('Drake:ContactWrenchVisualizer:t_knot should be numeric');
      end
      obj.t_knot = t_knot(:)';
      if(~isstruct(wrench_sol))
        error('Drake:ContactWrenchVisualizer:wrench_sol should be struct');
      end
      if(~isfield(wrench_sol,'body') || ~isfield(wrench_sol,'body_pts') || ~isfield(wrench_sol,'pts_pos') || ~isfield(wrench_sol,'force') || ~isfield(wrench_sol,'torque'))
        error('Drake:ContactWrenchVisualizer: wrench_sol does not have the desired fields');
      end
      obj.num_contact_bodies = size(wrench_sol,1);
      if(length(obj.t_knot) ~= size(wrench_sol,2))
        error('Drake:ContactWrenchVisualizer: wrench_sol should have %d columns',length(obj.t_knot));
      end
      obj.wrench_sol = wrench_sol;
      obj.wrench_lcmgl = LCMGLClient('wrench');
      obj.force_scaler = manip.getMass*9.81/3;
      obj.torque_scaler = manip.getMass*9.81/50;
      obj.display_dt = 0.033; % Otherwise there's just too much lcmgl
      obj.atlas = Atlas();
    end
    
    function draw(obj,t,y)
      t_ind1 = find(t>obj.t_knot,1,'last');
      if(isempty(t_ind1))
        t_ind1 = 1;
        t_ind2 = 1;
      else
        t_ind2 = t_ind1+1;
      end
      Ldot = [0;0;-obj.model.getMass*9.81];
      Hdot = zeros(3,1);
      kinsol = obj.model.doKinematics(y(1:obj.model.getNumPositions));
      com = obj.model.getCOM(kinsol);
      for i = 1:obj.num_contact_bodies
        pts_pos_i = obj.wrench_sol(i,t_ind1).pts_pos;
        force_i1 = obj.wrench_sol(i,t_ind1).force;
        torque_i1 = obj.wrench_sol(i,t_ind1).torque;
        force_i1_norm = sum(force_i1.^2,1);
        torque_i1_norm = sum(torque_i1.^2,1);
        force_i2 = obj.wrench_sol(i,t_ind2).force;
        torque_i2 = obj.wrench_sol(i,t_ind2).torque;
        force_i2_norm = sum(force_i2.^2,1);
        torque_i2_norm = sum(torque_i2.^2,1);
        for j = 1:size(pts_pos_i,2)
          if(force_i1_norm(j)>0.01&&force_i2_norm(j)>0.01)
            force_ij = force_i2(:,j);
          elseif(force_i1_norm(j)<=0.01 && force_i2_norm(j)>0.01)
            force_ij = zeros(3,1);
          elseif(force_i1_norm(j)>0.01 && force_i2_norm(j)<=0.01)
            force_ij = zeros(3,1);
          else
            force_ij = zeros(3,1);
          end
          if(norm(force_ij)>0.01)
            obj.drawArrow(pts_pos_i(:,j),force_ij/obj.force_scaler,0.03,0.05,0.01,[0,0,1]);
          end
          if(torque_i1_norm(j)>0.01 && torque_i2_norm(j)>0.01)
            torque_ij = torque_i2(:,j);
          elseif(torque_i1_norm(j)<=0.01 && torque_i2_norm(j) <= 0.01)
            torque_ij = zeros(3,1);
          elseif(torque_i1_norm(j)<=0.01 && torque_i2_norm(j) <= 0.01)
            torque_ij = zeros(3,1);
          else
            torque_ij = zeros(3,1);
          end
          if(norm(torque_ij)>0.01)
            obj.drawArrow(pts_pos_i(:,j),torque_ij/obj.torque_scaler,0.03,0.05,0.01,[0,0.8,0.5]);
          end
        end
        Ldot = Ldot+sum(force_i2,2);
        Hdot = Hdot+sum(cross(pts_pos_i(1:3,:)-bsxfun(@times,com,ones(1,size(pts_pos_i,2))),force_i2),2)+sum(torque_i2,2);
      end
      
      % Draw the rate of centroidal momenta
      if(~isempty(obj.force_scaler))
        obj.wrench_lcmgl.glColor3f(1,0,0);
        obj.wrench_lcmgl.sphere(com,0.05,20,20);
%         obj.wrench_lcmgl.text(com,'CoM',0,0);
        drawArrow(obj,com,Ldot/obj.force_scaler,0.03,0.05,0.01,[0,0,1]);
        drawArrow(obj,com,Hdot/obj.force_scaler,0.03,0.05,0.01,[0,0.8,0.5]);
      end
      
      obj.wrench_lcmgl.switchBuffers();
      if(isempty(obj.atlas))
%         draw@BotVisualizer(obj,t,y);
      else
        obj.atlas.getStateFrame().publish(t,y,'EST_ROBOT_STATE');
      end
    end
    
    function drawArrow(obj,start_pos,arrow,head_width,head_length,body_width,color)
      length = norm(arrow);
      obj.wrench_lcmgl.glLineWidth(2);
      obj.wrench_lcmgl.glPushMatrix();
      obj.wrench_lcmgl.glTranslated(start_pos(1),start_pos(2),start_pos(3));
      obj.wrench_lcmgl.glColor3f(color(1),color(2),color(3));
%             obj.wrench_lcmgl.glBegin(obj.wrench_lcmgl.LCMGL_LINES);
%             obj.wrench_lcmgl.glVertex3f(0,0,0);
%             obj.wrench_lcmgl.glVertex3f(force_ij(1)/obj.force_scaler,force_ij(2)/obj.force_scaler,force_ij(3)/obj.force_scaler);
%             obj.wrench_lcmgl.glEnd();
      % compute the rotation angle
      rotate_axis = cross([1;0;0],arrow/length);
      if(norm(rotate_axis)>0.01)
        rotate_angle = asin(norm(rotate_axis))/pi*180;
        rotate_axis = rotate_axis/norm(rotate_axis);
        obj.wrench_lcmgl.glRotated(rotate_angle,rotate_axis(1),rotate_axis(2),rotate_axis(3));
      end
      
      obj.wrench_lcmgl.glPushMatrix();
      obj.wrench_lcmgl.glRotated(90,0,1,0);
      obj.wrench_lcmgl.cylinder([0;0;0],body_width,body_width,length-head_length,20,20);
      obj.wrench_lcmgl.glTranslated(0,0,length-head_length);
      obj.wrench_lcmgl.cylinder([0;0;0],head_width,0,head_length,20,20);
      obj.wrench_lcmgl.glPopMatrix();
%             obj.wrench_lcmgl.drawArrow3d(norm(force_ij)/obj.force_scaler,0.03,0.03,0.01);
      obj.wrench_lcmgl.glPopMatrix();
    end
    
  end
end

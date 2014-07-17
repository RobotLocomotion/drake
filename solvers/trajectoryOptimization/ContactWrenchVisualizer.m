classdef ContactWrenchVisualizer < BotVisualizer
  % This is a hack to visualize the contact wrenches and the centroidal momenta, together
  % with the robot
  properties(SetAccess = protected)
    t_knot
    wrench_sol
    wrench_lcmgl = LCMGLClient('wrench');
  end
  
  properties(Access = protected)
    num_contact_bodies
    force_scaler
    torque_scaler
  end
  
  methods
    function obj = ContactWrenchVisualizer(manip,t_knot,wrench_sol,use_contact_shapes)
      if nargin <4 
        use_contact_shapes = false; 
      end
      obj = obj@BotVisualizer(manip,use_contact_shapes);
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
    end
    
    function draw(obj,t,y)
      t_ind1 = find(t>obj.t_knot,1,'last');
      if(isempty(t_ind1))
        t_ind1 = 1;
        t_ind2 = 1;
      else
        t_ind2 = t_ind1+1;
      end
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
            obj.wrench_lcmgl.glLineWidth(2);
            obj.wrench_lcmgl.glPushMatrix();
            obj.wrench_lcmgl.glTranslated(pts_pos_i(1,j),pts_pos_i(2,j),pts_pos_i(3,j));
            obj.wrench_lcmgl.glColor3f(0,0,1);
            obj.wrench_lcmgl.glBegin(obj.wrench_lcmgl{i}.LCMGL_LINES);
            obj.wrench_lcmgl.glVertex3f(0,0,0);
            obj.wrench_lcmgl.glVertex3f(force_ij(1)/obj.force_scaler,force_ij(2)/obj.force_scaler,force_ij(3)/obj.force_scaler);
            obj.wrench_lcmgl.glEnd();
            obj.wrench_lcmgl.glPopMatrix();
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
            obj.wrench_lcmgl.glLineWidth(2);
            obj.wrench_lcmgl.glPushMatrix();
            obj.wrench_lcmgl.glTranslated(pts_pos_i(1,j),pts_pos_i(2,j),pts_pos_i(3,j));
            obj.wrench_lcmgl.glColor3f(0,0.3,0.8);
            obj.wrench_lcmgl.glBegin(obj.wrench_lcmgl{i}.LCMGL_LINES);
            obj.wrench_lcmgl.glVertex3f(0,0,0);
            obj.wrench_lcmgl.glVertex3f(torque_ij(1)/obj.torque_scaler,torque_ij(2)/obj.torque_scaler,torque_ij(3)/obj.torque_scaler);
            obj.wrench_lcmgl.glEnd();
            obj.wrench_lcmgl.glPopMatrix();
          end
        end
      end
      obj.wrench_lcmgl.switchBuffers();
      draw@BotVisualizer(obj,t,y);
    end
  end
end

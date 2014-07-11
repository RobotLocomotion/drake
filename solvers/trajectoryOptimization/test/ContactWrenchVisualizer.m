classdef ContactWrenchVisualizer < BotVisualizer
  % This is a hack to visualize the contact wrenches and the centroidal momenta, together
  % with the robot
  properties(SetAccess = protected)
    t_knot
    wrench_sol
    wrench_lcmgl
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
      obj.wrench_lcmgl = cell(obj.num_contact_bodies,1);
      for i = 1:obj.num_contact_bodies
        obj.wrench_lcmgl{i} = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),sprintf('%s_wrench',obj.model.getBody(obj.wrench_sol(i,1).body).linkname));
      end
      obj.force_scaler = manip.getMass*9.81;
      obj.torque_scaler = manip.getMass*9.81;
    end
    
    function draw(obj,t,y)
      draw@BotVisualizer(obj,t,y);
      t_ind = find(t<=obj.t_knot,1,'first');
      for i = 1:obj.num_contact_bodies
        pts_pos_i = obj.wrench_sol(i,t_ind).pts_pos;
        force_i = obj.wrench_sol(i,t_ind).force;
        torque_i = obj.wrench_sol(i,t_ind).torque;
        force_i_norm = sum(force_i.^2,1);
        torque_i_norm = sum(torque_i.^2,1);
        for j = 1:size(pts_pos_i,2)
          if(force_i_norm(j)>0.01)
            obj.wrench_lcmgl{i}.glLineWidth(2);
            obj.wrench_lcmgl{i}.glPushMatrix();
            obj.wrench_lcmgl{i}.glTranslated(pts_pos_i(1,j),pts_pos_i(2,j),pts_pos_i(3,j));
            obj.wrench_lcmgl{i}.glColor3f(0,1,0);
            obj.wrench_lcmgl{i}.glBegin(obj.wrench_lcmgl{i}.LCMGL_LINES);
            obj.wrench_lcmgl{i}.glVertex3f(0,0,0);
            obj.wrench_lcmgl{i}.glVertex3f(force_i(1,j)/obj.force_scaler,force_i(2,j)/obj.force_scaler,force_i(3,j)/obj.force_scaler);
            obj.wrench_lcmgl{i}.glEnd();
            obj.wrench_lcmgl{i}.glPopMatrix();
          end
          if(torque_i_norm(j)>0.01)
          end
        end
        obj.wrench_lcmgl{i}.switchBuffers();
      end
    end
  end
end
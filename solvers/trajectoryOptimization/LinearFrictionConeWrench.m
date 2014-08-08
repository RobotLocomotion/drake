classdef LinearFrictionConeWrench < RigidBodyContactWrench
  % constrain the contact force to be in a linearized friction cone
  % The contact force f_contact = [edge1 edge2 ... edgeN] *[f1;f2;...;fN] where edgeK is
  % the K'th edge of the linearized friction cone, fK is the force paramter along the K'th
  % edge
  properties(SetAccess = protected)
    normal_dir % A 3 x obj.num_pts vector. The axis of the cone
    FC_edge % A 3 x num_edge double matrix. FC_edge(:,i) is the i'th edge of
            % the linearized friction cone in the world frame.
  end
  
  methods
    function obj = LinearFrictionConeWrench(robot,body,body_pts,FC_edge)
      % @param body       -- The index of contact body on the robot
      % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
      % of the contact point on the body frame. Every contact point shares the same friction
      % cone.
      % @param FC_edge    -- A 3 x num_edge double matrix. FC_edge(:,i) is the i'th edge of
      % the linearized friction cone in the WORLD frame
      obj = obj@RigidBodyContactWrench(robot,body,body_pts);
      FC_edge_size = size(FC_edge);
      if(~isnumeric(FC_edge_size) || length(FC_edge_size) ~= 2 || FC_edge_size(1) ~= 3)
        error('Drake:LinearFrictionConeWrench: FC_edge should be a 3 x num_edges numeric matrix');
      end
      obj.num_pt_F = FC_edge_size(2);
      obj.FC_edge = FC_edge;
      FC_edge_norm = sqrt(sum(obj.FC_edge.^2,1));
      FC_edge_normalized = obj.FC_edge./bsxfun(@times,FC_edge_norm,ones(3,1));
      obj.normal_dir = sum(FC_edge_normalized,2);
      obj.normal_dir = bsxfun(@times,ones(1,obj.num_pts),obj.normal_dir/norm(obj.normal_dir));
      obj.F_lb = zeros(obj.num_pt_F,obj.num_pts);
      obj.F_ub = inf(obj.num_pt_F,obj.num_pts);
      obj.contact_force_type = RigidBodyContactWrench.LinearFrictionConeType;
      obj.num_wrench_constraint = 0;
      obj.wrench_iCfun = [];
      obj.wrench_jCvar = [];
      obj.wrench_cnstr_lb = [];
      obj.wrench_cnstr_ub = [];
      obj.wrench_cnstr_name = {};
    end
    
    function [c,dc] = evalWrenchConstraint(obj,kinsol,F,slack)
      % return the constraint. There is no nonlinear constraint on the force paramter
      c = [];
      dc = zeros(0,obj.robot.getNumPositions+obj.num_pt_F*obj.num_pts+obj.num_slack);
    end
    
    function A = torque(obj)
      % Compute the torque at each contact point from the force parameter.
      % @retval A   -- A (3*num_pts) x (obj.num_pt_F*obj.num_pts) double matrix.
      % reshape(A*F(:),3,num_pts) are the torque at each contact point
      A = sparse(3*obj.num_pts,obj.num_pt_F*obj.num_pts);
    end
    
    function A = force(obj)
      % Compute the indivisual force from the force parameters F. The individual forces
      % are reshape(A*F,3,obj.num_pts)
      % @retval A    -- A (3*num_pts) x (obj.num_pt_F*obj.num_pts) double matrix
      A = kron(speye(obj.num_pts),obj.FC_edge);
    end
    
    function [pos,J] = contactPosition(obj,kinsol)
      % Returns the contact positions and its gradient w.r.t q
      % @param t        -- A double scalar, the time to evaluate the contact position
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A matrix with 3 rows. pos(:,i) is the i'th contact position
      % @retval J       -- A matrix of size prod(size(pos)) x nq. The gradient of pos
      % w.r.t q
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.body_pts,0);
    end
  end
  
  methods(Access=protected)
    function lincon = generateWrenchLincon(obj)
      lincon = LinearConstraint([],[],zeros(0,obj.num_pt_F*obj.num_pts));
    end
  end
end
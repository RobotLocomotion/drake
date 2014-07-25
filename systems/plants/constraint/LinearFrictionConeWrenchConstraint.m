classdef LinearFrictionConeWrenchConstraint < ContactWrenchConstraint
  % constrain the contact force to be in a linearized friction cone
  % The contact force f_contact = [edge1 edge2 ... edgeN] *[f1;f2;...;fN] where edgeK is
  % the K'th edge of the linearized friction cone, fK is the force paramter along the K'th
  % edge
  % @param body       -- The index of contact body on the robot
  % @param body_name  -- The name of the body.
  % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
  % of the contact point on the body frame. Every contact point shares the same friction
  % cone.
  % @param num_pts    -- A scalar. The number of contact points
  % @param num_edges  -- A scalar. The number of edges in one linearized friction cone
  % @param FC_edge    -- A 3 x num_edge double matrix. FC_edge(:,i) is the i'th edge of
  % the linearized friction cone in the world frame.
  properties(SetAccess = protected)
    body 
    body_name
    body_pts
    num_pts
    num_edges
    FC_edge
  end
  
  methods
    function obj = LinearFrictionConeWrenchConstraint(robot,body,body_pts,FC_edge,tspan)
      % @param body       -- The index of contact body on the robot
      % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
      % of the contact point on the body frame. Every contact point shares the same friction
      % cone.
      % @param FC_edge    -- A 3 x num_edge double matrix. FC_edge(:,i) is the i'th edge of
      % the linearized friction cone
      % @param tspan      -- A 1 x 2 double vector. The time span of the constraint being
      % active
      if(nargin<5)
        tspan = [-inf,inf];
      end
      obj = obj@ContactWrenchConstraint(robot,tspan);
      body_size = size(body);
      if(~isnumeric(body) || length(body_size) ~= 2 || body_size(1) ~= 1 || body_size(2) ~= 1)
        error('Drake:LinearFrictionConeWrenchConstraint: body should be a numeric scalar');
      end
      obj.body = body;
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      body_pts_size = size(body_pts);
      if(~isnumeric(body_pts) || length(body_pts_size) ~= 2 || body_pts_size(1) ~= 3)
        error('Drake:LinearFrictionConeWrenchConstraint: body_pts should be 3 x num_pts double matrix');
      end
      obj.body_pts = body_pts;
      obj.num_pts = body_pts_size(2);
      FC_edge_size = size(FC_edge);
      if(~isnumeric(FC_edge_size) || length(FC_edge_size) ~= 2 || FC_edge_size(1) ~= 3)
        error('Drake:LinearFrictionConeWrenchConstraint: FC_edge should be a 3 x num_edges numeric matrix');
      end
      obj.num_edges = FC_edge_size(2);
      obj.FC_edge = FC_edge;
      obj.pt_num_F = obj.num_edges;
      obj.F_lb = zeros(obj.pt_num_F,obj.num_pts);
      obj.F_ub = inf(obj.pt_num_F,obj.num_pts);
      obj.num_constraint = 0;
      obj.type = RigidBodyConstraint.LinearFrictionConeWrenchConstraintType;
    end
    
    function [c,dc_val] = evalSparse(obj,kinsol,F)
      % return the constraint. There is no nonlinear constraint on the force paramter
      c = [];
      dc_val = [];
    end
    
    function [iCfun,jCvar,nnz] = evalSparseStructure(obj)
      % return the sparse structure of the nonlinear constraint gradient. In this case the
      % gradient matrix is empty.
      iCfun = [];
      jCvar = [];
      nnz = 0;
    end
    
    function [lb,ub] = bounds(obj,t)
      % The lower and upper bound of the nonlinear constraint returned from eval. Here
      % they are empty
      lb = [];
      ub = [];
    end
    
    function [tau,dtau] = torqueSum(obj,kinsol,F)
      % Compute the total torque and its gradient      
      valid_F = checkForceSize(obj,F);
      if(~valid_F)
        error('Drake:LinearFrictionConeWrenchConstraint:friction force should be num_edges x n_pts matrix');
      end
      nq = obj.robot.getNumDOF();
      [body_pos,dbody_pos] = forwardKin(obj.robot,kinsol,obj.body,obj.body_pts,0);
      force = obj.FC_edge*F;
      tau = sum(cross(body_pos,force,1),2);
      dtau = zeros(3,nq+obj.num_pts*obj.num_edges);
      for i = 1:obj.num_pts
        dtau_tmp = dcross(body_pos(:,i),force(:,i));
        dtau(:,1:nq) = dtau(:,1:nq)+dtau_tmp(:,1:3)*dbody_pos((i-1)*3+(1:3),:);
        dtau(:,nq+(i-1)*obj.num_edges+(1:obj.num_edges)) = dtau_tmp(:,4:6)*obj.FC_edge; 
      end
    end
    
    function A = forceSum(obj)
      % Compute total force from contact force parameters F. The total force is a linear
      % transformation from F, total_force = A*F(:);
      A = repmat(obj.FC_edge,1,obj.num_pts);
    end
    
    function A = force(obj)
      % Compute the indivisual force from the force parameters F. The individual forces
      % are reshape(A*F,3,obj.num_pts)
      % @retval A    -- A (3*num_pts) x (obj.pt_num_F*obj.num_pts) double matrix
      iAfun = reshape(repmat(reshape(1:3*obj.num_pts,3,obj.num_pts),obj.num_edges,1),[],1);
      jAvar = reshape(bsxfun(@times,ones(3,1),(1:obj.num_pts*obj.num_edges)),[],1);
      Aval = reshape(bsxfun(@times,obj.FC_edge(:),ones(1,obj.num_pts)),[],1);
      A = sparse(iAfun,jAvar,Aval,3*obj.num_pts,obj.num_edges*obj.num_pts);
    end
    
    function name_str = name(obj,t)
      name_str = {};
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
end
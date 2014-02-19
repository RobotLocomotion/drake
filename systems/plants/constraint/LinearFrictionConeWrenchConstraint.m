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
  % the linearized friction cone
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
      if(~isnumeric(FC_axis) || length(FC_edge_size) ~= 2 || FC_edge_size(1) ~= 3)
        error('Drake:LinearFrictionConeWrenchConstraint: FC_edge should be a 3 x num_edges numeric matrix');
      end
      num_edges = FC_edge_size(2);
      % Find the convex edge of the linearized friction cone 
      K = convhull([0 FC_edge(1,:)]',[0 FC_edge(2,:)]',[0 FC_edge(3,:)]');
      
    end
  end
end
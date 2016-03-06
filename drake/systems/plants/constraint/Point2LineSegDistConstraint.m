classdef Point2LineSegDistConstraint < SingleTimeKinematicConstraint
  % This constrain the distance between a point on a body to a line segment is within a range.
  % Also the projection of the point is within the LineSegment
  % @param robot               
  % @param pt_body                -- The body index
  % @param pt                     -- A 3x1 double array, the coordinate of the point in
  %                                  pt_body frame
  % @param line_body              -- The body index, the body on which the line segment is
  %                                  on
  % @param line_ends              -- a 3x2 double matrix, line_ends(:,1) and
  %                                  line_ends(:,2) are the coordinate of the two ends of
  %                                  the line segment, on the line_body frame
  % @param dist_lb                -- A nonnegative double scalar. The lower bound of the
  %                                  point to line distance
  % @param dist_ub                -- A nonnegative double scalar. The upper bound of the
  %                                  point to line distance
  % @param tspan                  -- A 1x2 double array, optional argument. The time span
  %                                  of the constraint. Default is [-inf inf];
  properties(SetAccess=protected)
    pt_body
    pt
    line_body
    line_ends;
    dist_lb
    dist_ub
    pt_body_name;
    line_body_name;
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      [pt_pos,J_pt] = forwardKin(obj.robot,kinsol,obj.pt_body,obj.pt,0);
      [line_pos,J_line] = forwardKin(obj.robot,kinsol,obj.line_body,obj.line_ends,0);
      x0 = pt_pos;
      x1 = line_pos(:,1);
      x2 = line_pos(:,2);
      x21 = x2-x1;
      J21 = J_line(4:6,:)-J_line(1:3,:);
      x10 = x1-x0;
      J10 = J_line(1:3,:)-J_pt;
      t = -(x10'*x21/(x21'*x21));
      dtdx10 = -x21'/(x21'*x21);
      dtdx21 = -(x10'*(x21'*x21)-x10'*x21*2*x21')/(x21'*x21)^2;
      dtdq = dtdx10*J10+dtdx21*J21;
      h = x0-(x1+x21*t);
      dhdq = J_pt-(J_line(1:3,:)+J21*t+x21*dtdq);
      d = h'*h;
      dddq = 2*h'*dhdq;
      c = [d;t];
      dc = [dddq;dtdq];
    end
  end
  
  methods
    function obj = Point2LineSegDistConstraint(robot,pt_body,pt,line_body,line_ends,dist_lb,dist_ub,tspan)
      if(nargin == 7)
        tspan = [-inf inf];
      end
      
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      if(~isnumeric(pt_body))
        error('Point2LineSegDistanceConstraint: pt_body should be numeric');
      end
      sizecheck(pt_body,[1,1]);
      if(pt_body>robot.getNumBodies || pt_body<0)
        error('Point2LineSegDistanceConstraint: pt_body is not valid');
      end
      obj.pt_body = pt_body;
      if(~isnumeric(pt))
        error('pt should be numeric');
      end
      sizecheck(pt,[3,1]);
      obj.pt = pt;
      if(~isnumeric(line_body))
        error('Point2LineSegDistanceConstraint: line_body should be numeric');
      end
      sizecheck(line_body,[1,1]);
      if(line_body>robot.getNumBodies || line_body<0)
        error('Point2LineSegDistanceConstraint: line_body is not valid');
      end
      obj.line_body = line_body;
      if(~isnumeric(line_ends))
        error('line_ends should be numeric');
      end
      sizecheck(line_ends,[3,2]);
      obj.line_ends = line_ends;
      if(~isnumeric(dist_lb))
        error('Point2LineSegDistanceConstraint: lb should be numeric');
      end
      if(~isnumeric(dist_ub))
        error('Point2LineSegDistanceConstraint: ub should be numeric');
      end
      sizecheck(dist_lb,[1,1]);
      sizecheck(dist_ub,[1,1]);
      if(dist_lb<0 || dist_lb>dist_ub)
        error('Point2LineSegDistanceConstraint: lb should be nonnegative, ub should be no less than lb');
      end
      obj.dist_lb = dist_lb;
      obj.dist_ub = dist_ub;
      obj.num_constraint = 2;
      obj.pt_body_name = obj.robot.getBody(obj.pt_body).linkname;
      obj.line_body_name = obj.robot.getBody(obj.line_body).linkname;
      obj.type = RigidBodyConstraint.Point2LineSegDistConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.Point2LineSegDistConstraintType,robot.getMexModelPtr,pt_body,pt,line_body,line_ends,dist_lb,dist_ub,tspan);
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = [obj.dist_lb^2;0];
        ub = [obj.dist_ub^2;1];
      else
        lb = [];
        ub = [];
      end
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('Distance between %s pt to a line segment on %s',obj.pt_body_name,obj.line_body_name);...
          sprintf('Fraction of point projection onto line segment')};
        
      else
        name_str = {};
      end
    end
    
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(obj.pt_body,obj.line_body);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
classdef WorldPositionInFrameConstraint < WorldPositionConstraint
% @param robot            A RigidBodyManipulator, or a
%                         TimeSteppingRigidBodyManipulator
% @param body             The index of the body
% @param pts              A 3xn_pts matrix, pts(:,i) represents ith pts in
%                         the body
% @param lb, ub           Both are 3xn_pts matrices. [lb(:,i), ub(:,i)]
%                         represents the bounding box for the 
%                         position of pts(:,i) in the world frame
% @param T                Homogeneous transform of the frame in which the
%                         positions should be evaluated
% @param tspan            OPTIONAL argument. A 1x2 vector
  properties(SetAccess = protected)
    f_T_o
    o_T_f
  end
  
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
      pos = homogTransMult(obj.f_T_o,pos);
      J = reshape(obj.f_T_o(1:3,1:3)*reshape(J,3,[]),3*obj.n_pts,[]);
    end
  end
  
  methods
    function obj = WorldPositionInFrameConstraint(robot,body,pts,o_T_f,lb,ub,tspan)
      if(nargin < 7)
        tspan = [-inf,inf];
      end
      ptr = constructPtrWorldPositionInFrameConstraintmex(robot.getMexModelPtr,body,pts,o_T_f,lb,ub,tspan);
      obj = obj@WorldPositionConstraint(robot,body,pts,lb,ub,tspan);
      obj.o_T_f = o_T_f;
      obj.f_T_o = invHT(o_T_f);
      obj.mex_ptr = ptr;
    end
    
    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      updatePtrWorldPositionInFrameConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end
    
    function ptr = constructPtr(varargin)
      ptr = constructPtrWorldPositionInFrameConstraintmex(varargin{:});
    end

    function drawConstraint(obj,q,lcmgl)
      kinsol = doKinematics(obj.robot,q,false,false);
      pts_w = forwardKin(obj.robot,kinsol,obj.body,obj.pts);
      o_P_f = obj.o_T_f(1:3,4);
      lcmgl.glDrawAxes();
      for pt = pts_w
        lcmgl.glColor3f(0.25,0.25,0.25);
        lcmgl.sphere( pt, 0.02, 36, 36);
      end
      a = rotmat2axis(obj.o_T_f(1:3,1:3));
      lcmgl.glTranslated(o_P_f(1),o_P_f(2),o_P_f(3));
      lcmgl.glRotated(a(4)*180/pi,a(1),a(2),a(3));
      lcmgl.glDrawAxes();
      lcmgl.glColor4f(0,1,0,0.5);
      lcmgl.box((obj.lb+obj.ub)/2,obj.ub-obj.lb);
      lcmgl.switchBuffers();
    end
  end
end

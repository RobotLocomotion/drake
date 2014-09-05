classdef WorldPosition < drakeFunction.kinematic.RelativePosition
  methods
    function obj = WorldPosition(rbm,frame,varargin)
      % fcn = WorldPosition(rbm,frame,pts_in_frame) returns a
      %   drakeFunction.kinematic.RelativePosition object representing the 
      %   world-position of the given points in the given frame.
      %
      % fcn = WorldPosition(rbm,frame) returns a
      %   drakeFunction.kinematic.RelativePosition object representing the 
      %   world-position of the origin of the given frame
      %
      % @param rbm            -- RigidBodyManipulator object
      % @param frame          -- Body/frame name or frame id/body idx
      % @param pts_in_frame   -- [3 x n_pts] array. pts_in_frame(:,i) gives the
      %                          coordinates of the i-th point in the given frame.
      %                          Optional. @default [0;0;0]
      obj = obj@drakeFunction.kinematic.RelativePosition(rbm,frame,1,varargin{:});
    end
    
    function [pos,J] = eval(obj,kinsol)
      % pos = eval(obj,kinsol) returns the world positions of the points
      %
      % [pos,J] = eval(obj,kinsol) also returns the Jacobian of the world
      %   positions
      %
      % @param obj  -- drakeFunction.kinematic.RelativePosition object
      % @param kinsol  -- The kinsol struct returned from RigidBodyManipulator.doKinematics function
      if(obj.frameA == 0)
        [pos,J] = getCOM(obj.rbm,kinsol);
      else
        [pos,J] = forwardKin(obj.rbm,kinsol,obj.frameA,0);
        pos = pos(:);
      end
    end
  end
end

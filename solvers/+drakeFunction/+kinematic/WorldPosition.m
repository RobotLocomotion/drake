function fcn = WorldPosition(rbm,frame,varargin)
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
  fcn = drakeFunction.kinematic.RelativePosition(rbm,frame,1,varargin{:});
end

function fcn = WorldQuaternion(rbm,frame,varargin)
  % fcn = WorldQuaternion(rbm,frame) returns a
  %   drakeFunction.kinematic.RelativeQuaternion object representing the 
  %   world-orientation of the given frame.
  %
  % @param rbm            -- RigidBodyManipulator object
  % @param frame          -- Body/frame name or frame id/body idx
  fcn = drakeFunction.kinematic.RelativeQuaternion(rbm,frame,1);
end

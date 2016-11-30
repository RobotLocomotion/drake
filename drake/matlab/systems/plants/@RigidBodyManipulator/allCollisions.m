function [ptsA,ptsB,idxA,idxB] = allCollisions(obj,kinsol)
  % [ptsA,ptsB,idxA,idxB] = allCollisions(obj,kinsol) uses Bullet's
  % collision checking (including their broad-phase) to give a fast
  % report of all points of collision in the manipulator with the
  % following exceptions:
  %
  %   - any body and its parent in the kinematic tree (unless its parent
  %     is the world)
  %   - body A and body B, where body A belongs to collision filter
  %     groups that ignore all of the collision filter groups to which
  %     body B belongs and vice versa
  %
  % One probably shouldn't use this inside any optimization routines -
  % it returns a variable number of points and has not been checked for
  % numerical differentiability.
  %
  % @param kinsol  the output of doKinematics. Note that this method
  %   requires a kinsol with mex enabled
  %
  % @retval ptsA the i-th column contains the collision point (as
  %   identified by Bullet) on body A (in world coordinates) 
  % @retval ptsB the i-th column contains the collision point (as
  %   identified by Bullet) on body B (in world coordinates) 
  % @retval idxA link indices for the first body in each collision
  % @retval idxB link indices for the second body in each collision
  % @ingroup Collision

  if ~isstruct(kinsol)  
    % treat input as allCollisions(obj,q)
    kinsol = doKinematics(obj,kinsol);
  end

  if ~kinsol.mex 
    error('RigidBodyManipulator:allCollisions:doKinematicsMex', ...
      'You must call doKinematics with mex enabled');
  end

  [ptsA,ptsB,idxA,idxB] = allCollisionsmex(obj.mex_model_ptr, kinsol.mex_ptr);
end

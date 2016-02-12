function [phi,n,x,body_x, body_idx] = collisionDetectFromPoints(obj,kinsol,points,use_margins)
  % Given a set of points in global frame, returns the signed distance
  % of each point to the closest surface of the RBM. Utilizes the same
  % collision-checking machinery of collisionDetect(), but instead of
  % checking distances between objects in the model, computes distances
  % for the supplied points.
  %
  % @param kinsol  the output of doKinematics. Note that this method
  %   requires a kinsol with mex enabled
  % @param points  3xN pointcloud in global frame
  % @param use_margins  whether the collision will be checked against
  %   artificially-inflated objects (which have better gradients)
  %
  % @retval phi       Nx1     signed distances
  % @retval n         3xN     normals at nearest body
  % @retval x         3xN     points on clearest body
  % @retval body_x    3xN     points on clearest body
  % @retval body_idx  Nx1     body idx of nearest to each point
  % @ingroup Collision

  if ~kinsol.mex 
    error('RigidBodyManipulator:allCollisions:doKinematicsMex', ...
      'You must call doKinematics with mex enabled');
  end

  [phi, n, x, body_x, body_idx] = collisionDetectFromPointsmex(obj.mex_model_ptr, kinsol.mex_ptr, points, use_margins);
  body_idx = body_idx + 1;
end

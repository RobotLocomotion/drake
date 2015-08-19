function [x_in_world, J, dJ] = terrainContactPositions(obj, ...
    kinsol,varargin)
  % x_in_world = terrainContactPositions(obj,kinsol) returns the
  %   world-frame position of all terrain contact points on the
  %   manipulator.
  %
  % [x_in_world,J] = terrainContactPositions(obj,kinsol) also
  %   returns the derivative of those positions w.r.t the manipulator's
  %   position variables.
  %
  % [x_in_world,J,dJ] = terrainContactPositions(obj,kinsol) also
  %   returns the second derivative of those positions w.r.t the
  %   manipulator's position variables.
  %   
  % [...] = terrainContactPositions(obj,kinsol,body_idx)
  %   considers only the bodies indicated by body_idx.
  %
  % [...] = terrainContactPositions(obj,kinsol, ...
  %   terrain_contact_point_struct) considers the body-frame points
  %   specified by terrain_contact_point_struct.  See
  %   RigidBodyManipulator/getTerrainContactPoints for a description of
  %   terrain_contact_point_struct.
  %
  % [...] = terrainContactPositions(obj,kinsol,...,compute_Jdot_instead_of_dJ)
  %   returns the time derivative of J rather than dJ when second
  %   derivatives are requested.
  %
  % For a general description of terrain contact points see 
  % <a href="matlab:help RigidBodyGeometry/getTerrainContactPoints">RigidBodyGeometry/getTerrainContactPoints</a>
  %
  % @param obj - RigidBodyManipulator object
  % @param kinsol - Structure returned by doKinematics
  % @param body_idx - vector of body indices @default <All bodies>
  % @param terrain_contact_point_struct - See description in
  %   RigidBodyManipulator/getTerrainContactPoints
  %
  % @retval x_in_world - world-frame position of the terrain contact
  %                      points
  % @retval J - dx_in_world/dq
  % @retval dJ - d^2x_in_world/dq^2

  if numel(varargin) > 2
    compute_Jdot_instead_of_dJ = varargin{3};
    varargin = varargin(1:2);
  elseif numel(varargin) > 1 && islogical(varargin{2})
    compute_Jdot_instead_of_dJ = varargin{2};
    varargin = varargin(1);
  elseif numel(varargin) > 0 && islogical(varargin{1})
    compute_Jdot_instead_of_dJ = varargin{1};
    varargin = {};
  else
    compute_Jdot_instead_of_dJ = false;
  end

  compute_first_derivative = nargout > 1;
  compute_second_derivative = nargout > 2 && ~compute_Jdot_instead_of_dJ;
  compute_Jdot = nargout > 2 && compute_Jdot_instead_of_dJ;

  if compute_Jdot
    error('Computing Jdot is no longer supported. See terrainContactJacobianDotTimesV');
  end

  if ~isstruct(kinsol)
    kinsol = doKinematics(obj,kinsol,compute_second_derivative);
  end

  if numel(varargin) > 0 &&  isstruct(varargin{1})
    terrain_contact_point_struct = varargin{1};
  else
    terrain_contact_point_struct = getTerrainContactPoints(obj,varargin{:});
  end

  n_pts = arrayfun(@(x) size(x.pts,2),terrain_contact_point_struct);
  nq = obj.getNumPositions();
  cumsum_n_pts = cumsum([0,n_pts]);
  x_in_world = zeros(3,cumsum_n_pts(end))*kinsol.q(1);
  if compute_first_derivative
    J = zeros(3*cumsum_n_pts(end),nq)*kinsol.q(1);
  end
  if compute_second_derivative
    dJ = zeros(3*cumsum_n_pts(end),nq^2)*kinsol.q(1);
  end

  for i = 1:numel(terrain_contact_point_struct)
    if compute_second_derivative
      [x_in_world(:,cumsum_n_pts(i)+(1:n_pts(i))), ...
       J(3*cumsum_n_pts(i)+(1:3*n_pts(i)),:), ...
       dJ(3*cumsum_n_pts(i)+(1:3*n_pts(i)),:)] =  ...
        forwardKin(obj,kinsol,terrain_contact_point_struct(i).idx, ...
        terrain_contact_point_struct(i).pts);
    elseif compute_first_derivative
      [x_in_world(:,cumsum_n_pts(i)+(1:n_pts(i))), ...
       J(3*cumsum_n_pts(i)+(1:3*n_pts(i)),:)] =  ...
        forwardKin(obj,kinsol,terrain_contact_point_struct(i).idx, ...
        terrain_contact_point_struct(i).pts);
    else
      x_in_world(:,cumsum_n_pts(i)+(1:n_pts(i))) = ...
        forwardKin(obj,kinsol,terrain_contact_point_struct(i).idx, ...
        terrain_contact_point_struct(i).pts);
    end
  end
end

function [x_in_world, J, dJ_or_Jdot] = terrainContactPointsInWorld(obj, ...
    kinsol,varargin)
  % x_in_world = getTerrainContactPointsInWorld(obj,kinsol) returns the
  %   world-frame position of all terrain contact points on the
  %   manipulator.
  %
  % x_in_world = getTerrainContactPointsInWorld(obj,kinsol,body_idx)
  %   returns the world-frame position of all terrain contact points on
  %   the bodies indicated by body_idx.
  %
  % x_in_world = getTerrainContactPointsInWorld(obj,kinsol, ...
  %   terrain_contact_point_struct) returns the world-frame position of
  %   the body-frame points specified by terrain_contact_point_struct.
  %   See RigidBodyManipulator/getTerrainContactPoints for a description
  %   of terrain_contact_point_struct.
  %
  % @param obj - RigidBodyManipulator object
  % @param kinsol - Structure returned by doKinematics
  % @param body_idx - vector of body indices @default <All bodies>
  % @param terrain_contact_point_struct - See description in
  %   RigidBodyManipulator/getTerrainContactPoints
  
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

  if ~isstruct(kinsol)
    kinsol = doKinematics(obj,kinsol,compute_second_derivative);
  end

  if numel(varargin) > 0 &&  isstruct(varargin{1})
    terrain_contact_point_struct = varargin{1};
  else
    terrain_contact_point_struct = getTerrainContactPoints(obj,varargin{:});
  end

  n_pts = arrayfun(@(x) size(x.pts,2),terrain_contact_point_struct);
  nq = obj.getNumDOF();
  cumsum_n_pts = cumsum([0,n_pts]);
  x_in_world = zeros(3,cumsum_n_pts(end))*kinsol.q(1);
  if compute_first_derivative
    J = zeros(3*cumsum_n_pts(end),nq)*kinsol.q(1);
  end
  if compute_second_derivative
    dJ = zeros(3*cumsum_n_pts(end),nq^2)*kinsol.q(1);
  elseif compute_Jdot
    Jdot = zeros(3*cumsum_n_pts(end),nq)*kinsol.q(1);
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
      if compute_Jdot
        Jdot(3*cumsum_n_pts(i)+(1:3*n_pts(i)),:) = ...
          forwardJacDot(obj,kinsol, ...
                        terrain_contact_point_struct(i).idx, ...
                        terrain_contact_point_struct(i).pts);
      end
    else
      x_in_world(:,cumsum_n_pts(i)+(1:n_pts(i))) = ...
        forwardKin(obj,kinsol,terrain_contact_point_struct(i).idx, ...
        terrain_contact_point_struct(i).pts);
    end
  end

  if compute_second_derivative
    dJ_or_Jdot = dJ;
  elseif compute_Jdot_instead_of_dJ
    dJ_or_Jdot = Jdot;
  end
end

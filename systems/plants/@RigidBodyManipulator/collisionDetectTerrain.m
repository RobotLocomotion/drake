function [phi,normal,xB,idxB] = collisionDetectTerrain(obj, varargin)
  % [phi,normal,xB,idxB] = collisionDetectTerrain(obj, xA_in_world) 
  %   returns collision information for the world-frame points specified
  %   by xA_in_world.
  %
  % [...] = collisionDetectTerrain(obj, kinsol, terrain_contact_point_struct) 
  %   returns collision information for the body-frame points specified
  %   by terrain_contact_point_struct in the configuration specified by
  %   kinsol.
  %
  %   @param obj - RigidBodyManipulator object
  %   @param xA_in_world - 3 x m array of points in world-frame
  %   @param kinsol - structure returned by doKinematics
  %   @retval terrain_contact_point_struct - n x 1 structure array, where
  %     n is the number of bodies with points that can collide with
  %     terrain. Each element has the following fields
  %       * idx - Index of a body in the RigidBodyManipulator
  %       * pts - 3 x k array containing points on the body specified by
  %               idx that can collide with non-flat terrain.
  if nargin == 2
    xA_in_world = varargin{1};
  elseif nargin == 3
    kinsol = varargin{1};
    terrain_contact_point_struct = varargin{2};
    xA_in_world = terrainContactPointsInWorld(obj,kinsol, ...
                    terrain_contact_point_struct);
  else
    error('Drake:RigidBodyManipulator:collisionDetectTerrain:nargin', ...
          ['Invalid number of input arguments. See help for ' ...
           'RigidBodyManipulator/collisionDetectTerrain.']);
  end
  [z,normal] = getHeight(obj.terrain,xA_in_world(1:2,:));
  xB = [xA_in_world(1:2,:);z];
  idxB = ones(1,size(xA_in_world,2));
  %phi = sqrt(sum((xA_in_world-xB).^2))';
  phi = (xA_in_world(3,:)-z)';
end

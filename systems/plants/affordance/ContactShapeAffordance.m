classdef ContactShapeAffordance < ContactAffordance
  properties
    group_ind;  % Index of the collision group in 
                % obj.robot.body(obj.body_ind).collision_group_name
    collision_ind = 1; % Index of the collision geometry within the collision group
    T_geom_to_link;
  end
  methods
    function obj = ContactShapeAffordance(robot,body_ind,groupname,collision_ind,is_fixed)
      parent_args = {};
      if nargin > 0
        parent_args = {robot,body_ind};
      end
      if nargin < 3
        groupname = 'default';
      end
      if nargin > 4
        parent_args = {parent_args,is_fixed};
      end
      obj = obj@ContactAffordance(parent_args{:});
      if nargin > 3
        obj.collision_ind = collision_ind;
      end
      obj.group_ind = find(strcmp(groupname,obj.robot.body(obj.body_ind).collision_group_name));
      obj.T_geom_to_link = ...
        obj.robot.body(obj.body_ind).getContactShapes(obj.group_ind, ...
                                                      obj.collision_ind).T;
    end

    function [T_geom_to_world,T_link_to_world,dT_g2wdq] = getTransform(obj)
      % Returns the transform from collision geometry frame to world frame.
      T_link_to_world = obj.robot.body(obj.body_ind).T;
      T_geom_to_link = T_link_to_world*obj.T_geom_to_link;
      dTg2w_dq = obj.robot.body(obj.body_ind).dTdq*obj.T_geom_to_link;
    end

    function [pts_geom, J_geom] = inGeomFrame(obj,r_world,J_world)
      % Returns the vector r expressed in the collision geometry frame, where
      % pts_world is the same vector r expressed in the world frame.
      npts = size(r_world,2);
      nq = obj.robot.getNumPositions();
      if nargout < 2 
        [T_geom_to_world] = getTransform(obj);
      else
        [T_geom_to_world,T_link_to_world,dTg2w_dq] = getTransform(obj);
      end
      pts_geom = T_geom_to_world\[r_world;1+0*r_world(1,:)];
      if nargout > 1
        assert(nargin==3, 'Cannot generate J_geom without knowing J_world');
        J_world_full = reshape([reshape(J_world',3*nq,npts);zeros(nq,npts)],nq,4*npts)';
        J_geom_full = inv(T_geom_to_world)*(J_world - reshape(dTg2w_dq*pts_geom,nq,[])');
        J_geom = J_geom_full(mod(1:4*npts,4)~=0);
      end
      pts_geom = bsxfun(@rdivide,y(1:end-1,:),y(end,:)); % this is faster than repmat
    end
  end
end

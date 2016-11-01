classdef RigidBodySupportState
  %RigidBodySupportState defines a set of supporting bodies and contacts for a rigid
  %  body manipulator

  properties (SetAccess=protected)
    bodies; % array of supporting body indices
    contact_pts; % cell array of supporting contact points, each cell is 3x[num pts per body]
    contact_groups; % cell array of cell arrays of contact group strings, 1 for each body
    num_contact_pts;  % convenience array containing the desired number of
                      %             contact points for each support body
    support_surface; % 4-vector describing a support surface: [v; b] such that v' * [x;y;z] + b == 0
   end

  methods
    function obj = RigidBodySupportState(r,bodies,options)
      typecheck(r,'Biped');
      typecheck(bodies,'double');
      if nargin < 3
        options = struct();
      end
      
      obj.bodies = bodies(bodies~=0);
      nbod = length(obj.bodies);
      obj.num_contact_pts = zeros(nbod,1);
      obj.contact_pts = cell(1,nbod);
      obj.contact_groups = {};
      
      if isfield(options,'contact_groups')
        typecheck(options.contact_groups,'cell');
        sizecheck(options.contact_groups,nbod);
        for i=1:nbod
          body = r.getBody(obj.bodies(i));
          body_groups = options.contact_groups{i};
          obj.contact_pts{i} = body.getTerrainContactPoints(body_groups);
          obj.contact_groups{i} = options.contact_groups{i};
          obj.num_contact_pts(i)=size(obj.contact_pts{i},2);
        end
      else
        % use all points on body
        for i=1:length(obj.bodies)
          terrain_contact_point_struct = getTerrainContactPoints(r,obj.bodies(i));
          obj.contact_pts{i} = terrain_contact_point_struct.pts;
          obj.contact_groups{i} = r.getBody(obj.bodies(i)).collision_geometry_group_names;
          obj.num_contact_pts(i)=size(obj.contact_pts{i},2);
        end
      end
      
      if isfield(options,'use_support_surface')
        warning('Drake:RigidBodySupportState:UseSupportSurfaceDepcreated', 'The use_support_surface option has been deprecated. A support surface is now mandatory');
      end

      if isfield(options,'support_surface')
        typecheck(options.support_surface,'cell');
        sizecheck(options.support_surface,nbod);
        obj.support_surface = options.support_surface;
      else
        warning('Drake:RigidBodySupportState:NoSupportSurface', 'No support surface provided. A horizontal plane at z = 0 will be assumed');
        obj.support_surface = cell(1,nbod);
        for i=1:nbod
          obj.support_surface{i} = [0;0;1;0];
        end
      end
    end

    function obj = setContactPts(obj, ind, contact_pts, contact_groups)
      obj.contact_pts{ind} = contact_pts;
      obj.contact_groups{ind} = contact_groups;
      obj.num_contact_pts(ind) = size(contact_pts, 2);
    end
  end

end

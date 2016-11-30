classdef RigidBody < RigidBodyElement
  
  properties 
    robotnum = 0;  % this body is associated with a particular robot/object number, named in model.name{objnum} 
    
    % link properties
    linkname='';  % name of the associated link
    % NOTE: dofnum is deprecated, use position_num or velocity_num instead
    position_num=0;     % the indices into the joint configuration (q) vector corresponding to this joint
    velocity_num=0;     % the indices into the joint velocity (v) vector corresponding to this joint
    gravity_off=false;
    
    visual_geometry={}; % objects of type RigidBodyGeometry
    collision_geometry={}; % objects of type RigidBodyGeometry

    collision_geometry_group_names={};  % string names of the groups
    collision_geometry_group_indices={}; % collision_geometry_group_indices{i} is a list of indices into collision_geometry which belong to collision_geometry_group_names{i}
    
    % joint properties
    parent=0;       % index (starting at 1) for rigid body parent.  0 means no parent
    jointname='';
    pitch=0;        % for featherstone 3D models
    floating=0; % 0 = not floating base, 1 = rpy floating base, 2 = quaternion floating base
    joint_axis=[1;0;0]; 
    Ttree=eye(4);   % homogeneous transform from joint predecessor frame to parent body, so that the transform from this body to its parent is body.Ttree * jointTransform(body, q_body)
    damping=0; % viscous friction term
    coulomb_friction=0; 
    static_friction=0; % currently not used for simulation
    coulomb_window=eps; % the threshold around zero velocity used for the PWL friction model (See Khalil and Dombre, Fig. 9.2(d))
    joint_limit_min=[];
    joint_limit_max=[];
    effort_min=[];
    effort_max=[];
    velocity_limit=[];
    has_position_sensor=false;
  end
  
  properties (SetAccess=protected, GetAccess=public)    
    % mass, com, inertia properties
    I=zeros(6);  % total spatial mass matrix, sum of mass, inertia, (and added mass for submerged bodies)
    Imass=zeros(6);  % spatial mass/inertia
    Iaddedmass = zeros(6); % added mass spatial matrix
    mass = 0;
    com = [];
    inertia = [];

    % Collision filter properties
    collision_filter = struct('belongs_to',CollisionFilterGroup.DEFAULT_COLLISION_FILTER_GROUP_ID, ...
                             'ignores',CollisionFilterGroup.NO_COLLISION_FILTER_GROUPS);
  end
  
  methods    

    function q_body = getRandomConfiguration(body)
      if body.floating == 0
        if isinf(body.joint_limit_min) && isinf(body.joint_limit_max)
          q_body = randn;
        elseif ~isinf(body.joint_limit_min) && ~isinf(body.joint_limit_max)
          q_body = body.joint_limit_min + rand * (body.joint_limit_max - body.joint_limit_min);
        elseif ~isinf(body.joint_limit_min)
          % one-sided joint limit
          q_body = randn;
          if (q_body<body.joint_limit_min) % then flip it to the other side of the limit
            q_body = body.joint_limit_min + (body.joint_limit_min - q_body);
          end
        else
          % other one-sided joint limit
          q_body = randn;
          if (q_body>body.joint_limit_max) % then flip it to the other side of the limit
            q_body = body.joint_limit_max - (q_body - body.joint_limit_max);
          end
        end
      else
        pos = randn(3, 1);
        axis_angle = [randn(3, 1); (rand - 0.5) * 2 * pi];
        if body.floating == 1
          q_body = [pos; axis2rpy(axis_angle)];
        elseif body.floating == 2
          q_body = [pos; axis2quat(axis_angle)];
        else
          error('floating joint type not recognized');
        end
      end
    end
    
    function q_body = getZeroConfiguration(body)
      if body.floating == 2
        q_body = [zeros(3, 1); 1; 0; 0; 0];
      else
        q_body = zeros(length(body.position_num), 1);
      end
    end
    
    function varargout = forwardKin(varargin)
      error('forwardKin(body,...) has been replaced by forwardKin(model,body_num,...), because it has a mex version.  please update your kinematics calls');
    end

    function pts = getTerrainContactPoints(body,collision_group)
      % pts = getTerrainContactPoints(body) returns the terrain contact
      % points of all geometry on this body, in body frame.
      %
      % pts = getTerrainContactPoints(body,collision_group) returns the
      % terrain contact points of all geometry on this body belonging
      % to the group[s] specified by collision_group
      %
      % For a general description of terrain contact points see 
      % <a href="matlab:help RigidBodyGeometry/getTerrainContactPoints">RigidBodyGeometry/getTerrainContactPoints</a>
      %
      % @param body - A RigidBody object
      % @param collision_group - A string or cell array of strings
      %                          specifying the collision groups whose
      %                          terrain contact points should be
      %                          returned
      % @retval pts - A 3xm array of points on body (in body frame) that can collide with
      %               non-flat terrain
      if nargin < 2
        pts = cell2mat(cellfun(@(geometry) geometry.getTerrainContactPoints(), ...
                               body.collision_geometry, ...
                               'UniformOutput',false));
      else
        typecheck(collision_group,{'char','cell'});
        pts = cell2mat(cellfun(@(geometry) geometry.getTerrainContactPoints(), ...
          body.getCollisionGeometry(collision_group), ...
          'UniformOutput',false));
      end
    end
    
    function [pts,inds] = getContactPoints(body,collision_group)
      error('contact points have been replaced by collision geometry');
    end

    function dofnum(obj)
      error('the dofnum parameter is no longer supported, use position_num and velocity_num instead');
    end
    
    function varargout = getContactShapes(varargin)
      errorDeprecatedFunction('getCollisionGeometry');
    end
    
    function geometry = getCollisionGeometry(body,collision_group,collision_ind)
      % @param collision_group (optional) return structures for only the
      % collision_geometry in that group.  can be an integer index or a string.
      if (nargin<2) 
        geometry = body.collision_geometry;
      else
        if ~isnumeric(collision_group)
          typecheck(collision_group,{'char','cell'});
          collision_group = find(ismember(body.collision_geometry_group_names,collision_group));
        end
        if (nargin < 3)
          geometry = body.collision_geometry([body.collision_geometry_group_indices{collision_group}]);
        else
          geometry = body.collision_geometry(body.collision_geometry_group_indices{collision_group}(collision_ind));
        end
      end
    end

    function [body,body_changed] = replaceCylindersWithCapsules(body)
      % [body,body_changed] = replaceCylindersWithCapsules(body) returns
      % the body with all RigidBodyCylinders in collision_geometry replaced
      % by RigidBodyCapsules of the same dimensions.
      %
      % @param body - RigidBody object
      %
      % @retval body - RigidBody object
      % @retval body_changed - Logical indicating whether any
      %                        replacements were made.
      %
      cylinder_idx = cellfun(@(geometry) isa(geometry,'RigidBodyCylinder'), ...
                             body.collision_geometry);
      if ~any(cylinder_idx)
        body_changed = false;
      else
        body.collision_geometry(cylinder_idx) = ...
          cellfun(@(geometry) geometry.toCapsule(), ...
                  body.collision_geometry(cylinder_idx), ...
                  'UniformOutput',false);
        body_changed = true;
      end
    end
     
    function varargout = replaceContactShapesWithCHull(varargin)
      errorDeprecatedFunction('replaceCollisionGeometryWithConvexHull');
    end
    
    function body = replaceCollisionGeometryWithConvexHull(body,scale_factor)
      pts = [];
      for i = 1:length(body.collision_geometry)
        pts = [pts, body.collision_geometry{i}.getPoints()];
      end
      if ~isempty(pts)
        pts =pts(:,unique(convhull(pts')));
        if nargin > 1
          mean_of_pts = mean(pts,2);
          pts = bsxfun(@plus,scale_factor*bsxfun(@minus,pts,mean_of_pts),mean_of_pts);
        end
        body.collision_geometry = {};
        body.collision_geometry_group_indices = {};
        body.collision_geometry_group_names = {};
        body = body.addCollisionGeometry(RigidBodyMeshPoints(pts));
      end
    end

    function pts = getAxisAlignedBoundingBoxPoints(obj)
      % pts = getAxisAlignedBoundingBoxPoints(obj) returns the vertices of a
      % conservative, axis-aligned bounding-box of this body's collision
      % geometry. Specifically, it finds the bounding box of the bounding boxes
      % of all the geometries.
      %
      % @retval pts  -- 3-by-8 array of vertices
      pts = [];
      for i = 1:length(obj.collision_geometry)
        pts = [pts, obj.collision_geometry{i}.getBoundingBoxPoints()];
      end
      max_vals = repmat(max(pts,[],2),1,8);
      min_vals = repmat(min(pts,[],2),1,8);
      min_idx = logical([ 0 1 1 0 0 1 1 0;
                          1 1 1 1 0 0 0 0;
                          1 1 0 0 0 0 1 1]);
      pts = zeros(3,8);
      pts(min_idx) = min_vals(min_idx);
      pts(~min_idx) = max_vals(~min_idx);
    end
    
    function body = removeCollisionGroups(body,contact_groups)
      if isempty(body.collision_geometry), 
        return; 
      end % nothing to do for this body
      if ~iscell(contact_groups), 
        contact_groups={contact_groups}; 
      end
      for i=1:length(contact_groups)
        % boolean identifying if this collision_geometry_group_indices is being removed
        group_elements = strcmpi(contact_groups{i},body.collision_geometry_group_names);
        if ~isempty(group_elements)
          % indices of the body.collision_geometry that are being removed
          contact_inds = [body.collision_geometry_group_indices{group_elements}];
          
          if ~isempty(contact_inds)
            % indices of collision geometry that are *not* being removed
            keep_inds = setdiff(1:length(body.collision_geometry),contact_inds);
            
            % for each collision_geometry_group_indices, need to re-assign collision geometry
            % indices, since some are being removed
            ripts = nan(1,length(body.collision_geometry));  % reverse index
            ripts(keep_inds) = 1:length(keep_inds);
            for j=1:length(body.collision_geometry_group_indices)
              body.collision_geometry_group_indices{j}=ripts(body.collision_geometry_group_indices{j});
            end
            % remove collision geometry
            body.collision_geometry(:,contact_inds) = [];
          end
          % remove collision_geometry_group_indicess and names
          body.collision_geometry_group_indices(group_elements) = [];          
          body.collision_geometry_group_names(group_elements) = [];
        end
      end
    end
        
    function body = removeCollisionGroupsExcept(body,contact_groups)
      if isempty(body.collision_geometry), 
        return; 
      end % nothing to do for this body
      if ~iscell(contact_groups), 
        contact_groups={contact_groups}; 
      end
      i=1;
      while i<=length(body.collision_geometry_group_indices)
        %check of body.collision_geometry_group_names should not be kept
        if ~ismember(body.collision_geometry_group_names{i},contact_groups)
          % indices of the body.collision_geometry that are being removed
          contact_inds = [body.collision_geometry_group_indices{i}];
          
          if ~isempty(contact_inds)
            % indices of collision geometry that are *not* being removed
            keep_inds = setdiff(1:length(body.collision_geometry),contact_inds);
            
            % for each collision_geometry_group_indices, need to re-assign collision geometry
            % indices, since some are being removed
            ripts = nan(1,length(body.collision_geometry));  % reverse index
            ripts(keep_inds) = 1:length(keep_inds);
            for j=1:length(body.collision_geometry_group_indices)
              body.collision_geometry_group_indices{j}=ripts(body.collision_geometry_group_indices{j});
            end
            % remove collision geometry
            body.collision_geometry(:,contact_inds) = [];
          end
          % remove collision_geometry_group_indicess and names
          body.collision_geometry_group_indices(i) = [];          
          body.collision_geometry_group_names(i) = [];
        else
          i=i+1;
        end
      end
    end

    function body = makeBelongToNoCollisionFilterGroups(body)
      body.collision_filter.belongs_to = CollisionFilterGroup.NO_COLLISION_FILTER_GROUPS;
    end
    
    function body = makeIgnoreNoCollisionFilterGroups(body)
      body.collision_filter.ignores = CollisionFilterGroup.NO_COLLISION_FILTER_GROUPS;
    end

    function body = makeBelongToCollisionFilterGroup(body,collision_fg_id)
      for id = reshape(collision_fg_id,1,[])
        body.collision_filter.belongs_to(collision_fg_id) = true;
      end
    end
   
    function body = makeIgnoreCollisionFilterGroup(body,collision_fg_id)
      for id = reshape(collision_fg_id,1,[])
        body.collision_filter.ignores(collision_fg_id) = true;
      end
    end

    function newbody = copy(body)
      % makes a shallow copy of the body
      % note that this functionality is in matlab.mixin.Copyable in newer
      % versions of matlab, but I've done it myself since i want to support
      % < v2011
      
      newbody=RigidBody();
      p=properties(body);
      for i=1:length(p)
        eval(['newbody.',p{i},'=body.',p{i}]);
      end
    end
    
    % Note: bindParams and updateParams are copies of the methods in 
    % RigidBodyElement (yuck!) because the RigidBodyElement version does 
    % not have permissions to do the reflection on the protected properties 
    % in this class.
    % Also have some additional logic for the nested geometry elements
    function body=bindParams(body,model,pval)
      fr = getParamFrame(model);
      pn = properties(body);
      for i=1:length(pn)
        if isa(body.(pn{i}),'msspoly')
          body.param_bindings.(pn{i}) = body.(pn{i});
          if isnumeric(pval)
            body.(pn{i}) = double(subs(body.(pn{i}),fr.getPoly,pval));
          else
            body.(pn{i}) = subs(body.(pn{i}),fr.getPoly,pval);
          end            
        end
      end
      for i=1:length(body.visual_geometry)
        body.visual_geometry{i}=bindParams(body.visual_geometry{i},model,pval);
      end
      for i=1:length(body.collision_geometry)
        body.collision_geometry{i}=bindParams(body.collision_geometry{i},model,pval);
      end
    end
    
    function body=updateParams(body,poly,pval)
      % this is only intended to be called from the parent manipulator
      % class. (maybe I should move it up to there?)
      fn = fieldnames(body.param_bindings);
      for i=1:length(fn)
        body.(fn{i}) = double(subs(body.param_bindings.(fn{i}),poly,pval));
      end
      
      for i=1:length(body.visual_geometry)
        body.visual_geometry{i}=updateParams(body.visual_geometry{i},poly,pval);
      end
      for i=1:length(body.collision_geometry)
        body.collision_geometry{i}=updateParams(body.collision_geometry{i},poly,pval);
      end
    end
    

    
    function body = setInertial(body,varargin)
      % setInertial(body,mass,com,inertia [,addedmass]) or setInertial(body,spatialI [,addedmass])
      % this guards against things getting out of sync
      % Updated 4/5/2014 to allow for added mass effects (for submerged bodies)
      
      function v = skew(A)
        v = 0.5 * [ A(3,2) - A(2,3);
          A(1,3) - A(3,1);
          A(2,1) - A(1,2) ];
      end
      
      if nargin==2 || nargin==3
        sizecheck(varargin{1},[6 6]);
        % extract mass, center of mass, and inertia information from the
        % spatial I matrix
        body.Imass = varargin{1};
        body.mass = body.Imass(6,6);
        mC = body.Imass(1:3,4:6);
        body.com = skew(mC)/body.mass;
        body.inertia = body.Imass(1:3,1:3) - mC*mC'/body.mass;
        if nargin==3
            % Set added mass matrix
            sizecheck(varargin{2},[6 6]);
            body.Iaddedmass = varargin{2};
        end
        
      elseif nargin==4 || nargin==5
          % Set mass, center of mass, and inertia directly
        sizecheck(varargin{1},1);
        sizecheck(varargin{2},[3 1]);
        sizecheck(varargin{3},[3 3]);
        body.mass = varargin{1};
        body.com = varargin{2};
        body.inertia = varargin{3};
        
        C = vectorToSkewSymmetric(body.com);
        body.Imass = [ body.inertia + body.mass*C*C', body.mass*C; body.mass*C', body.mass*eye(3) ];
        
        if nargin==5
            % Set added mass matrix
            sizecheck(varargin{4},[6 6]);
            body.Iaddedmass = varargin{4};
        end
        
      else
        error('wrong number of arguments');
      end
      body.I = body.Imass+body.Iaddedmass;
      
      if isnumeric(body.I) && ~valuecheck(body.I'-body.I,zeros(6)); %Check symmetry of matrix
          warning('Spatial mass matrix is not symmetric, this is non-physical');
      end
    end    
        


    function varargout = addContactShape(varargin)
      errorDeprecatedFunction('addCollisionGeometry')
    end
    
    function body = addCollisionGeometry(body,geometry,name)
      if nargin < 3, name='default'; end
      geometry.name = name;
      body.collision_geometry = [body.collision_geometry,{geometry}];
      ind = find(strcmp(body.collision_geometry_group_names,name));
      if isempty(ind)
        body.collision_geometry_group_names=horzcat(body.collision_geometry_group_names,name);
        ind=length(body.collision_geometry_group_names);
        body.collision_geometry_group_indices{ind} = length(body.collision_geometry);
      else
        body.collision_geometry_group_indices{ind} = [body.collision_geometry_group_indices{ind},length(body.collision_geometry)];
      end
    end
  end
  
  methods (Static)
    % Deprecated properties
    function varargout = contact_pts(varargin)
      errorDeprecatedFunction('collision_geometry');
    end
    function varargout = visual_shapes(varargin)
      errorDeprecatedFunction('visual_geometry');
    end
    function varargout = contact_shapes(varargin)
      errorDeprecatedFunction('collision_geometry');
    end
    function varargout = contact_shape_group_name(varargin)
      errorDeprecatedFunction('collision_geometry_group_names');
    end
    function varargout = contact_shape_group(varargin)
      errorDeprecatedFunction('collision_geometry_group_indices');
    end
    
    function testRemoveCollisionGroups
      body = RigidBody();
      geometry1 = RigidBodySphere(1);
      geometry2 = RigidBodySphere(2);
      geometry3 = RigidBodySphere(3);
      geometry4 = RigidBodySphere(3);
      geometry5 = RigidBodySphere(3);
      body.collision_geometry = {geometry1, geometry2, geometry3, geometry4, geometry5};
      body.collision_geometry_group_names = {'group1','group2','group3'};
      body.collision_geometry_group_indices = {[1 4],2,[3 5]};
      body2 = body.removeCollisionGroups('group1');
      body3 = body.removeCollisionGroups('group2');
      body4 = body.removeCollisionGroups('group2adfs');
      
      assert(isequal(body2.collision_geometry,{geometry2, geometry3, geometry5}));
      assert(isequal(body2.collision_geometry_group_names,{'group2','group3'}));
      assert(isequal(body2.collision_geometry_group_indices,{[1], [2 3]}));
      
      assert(isequal(body3.collision_geometry,{geometry1, geometry3, geometry4, geometry5}));
      assert(isequal(body3.collision_geometry_group_names,{'group1','group3'}));
      assert(isequal(body3.collision_geometry_group_indices,{[1 3], [2 4]}));
      
      assert(isequal(body,body4));
      
      body5 = body.removeCollisionGroupsExcept({'group2','group3'});
      body6 = body.removeCollisionGroupsExcept({'group1','group3'});
      body7 = body.removeCollisionGroupsExcept({'group1'});
      
      assert(isequal(body7.collision_geometry,{geometry1, geometry4}));
      assert(isequal(body7.collision_geometry_group_names,{'group1'}));
      assert(isequal(body7.collision_geometry_group_indices,{[1 2]}));
      
      assert(isequal(body2,body5));
      assert(isequal(body3,body6));
      
    end
    
    function testMakeBelongToCollisionFilterGroup
      body = RigidBody();
      collision_fg_id = uint16(3);
      belongs_to_ref = '0000000000000101';
      body = makeBelongToCollisionFilterGroup(body,collision_fg_id);
      belongs_to = dec2bin(body.collision_filter.belongs_to,16);
      assert(strcmp(belongs_to,belongs_to_ref), ...
      'Expected ''%s'', found ''%s''',belongs_to_ref,belongs_to);
    end

    function testMakeIgnoreCollisionFilterGroup
      body = RigidBody();
      collision_fg_id = uint16(3);
      collides_with_ref = '1111111111111011';
      body = makeIgnoreCollisionFilterGroup(body,collision_fg_id);
      collides_with = dec2bin(body.collision_filter.collides_with,16);
      assert(strcmp(collides_with,collides_with_ref), ...
      'Expected ''%s'', found ''%s''',collides_with_ref,collides_with);
    end

    function testMakeBelongToNoCollisionFilterGroups
      body = RigidBody();
      collision_fg_id = uint16(3);
      belongs_to_ref = '0000000000000000';
      body = makeBelongToCollisionFilterGroup(body,collision_fg_id);
      body = makeBelongToNoCollisionFilterGroups(body);
      belongs_to = dec2bin(body.collision_filter.belongs_to,16);
      assert(strcmp(belongs_to,belongs_to_ref), ...
      'Expected ''%s'', found ''%s''',belongs_to_ref,belongs_to);
    end

    function testMakeIgnoreNoCollisionFilterGroups
      body = RigidBody();
      collision_fg_id = uint16(3);
      collides_with_ref = '1111111111111111';
      body = makeIgnoreCollisionFilterGroup(body,collision_fg_id);
      body = makeIgnoreNoCollisionFilterGroups(body);
      collides_with = dec2bin(body.collision_filter.collides_with,16);
      assert(strcmp(collides_with,collides_with_ref), ...
      'Expected ''%s'', found ''%s''',collides_with_ref,collides_with);
    end
  end
  
  methods    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.parent = map_from_old_to_new(obj.parent);
    end
  end
  
  methods (Static=true)
    function b = ConstantDensityBox(mass,size)
      % create a constant density box
      % @param total mass (kg)
      % @param size 3x1 vector w/ length, width, height
      
      b = RigidBody();
      b = setInertial(b,mass,zeros(3,1),mass/12*diag([size(2)^2+size(3)^2,size(1)^2+size(2)^2,size(1)^2+size(2)^2]));
      geom = RigidBodyBox(size);
      b.visual_geometry{1} = geom;
      b.collision_geometry{1} = geom;
    end
  end
end

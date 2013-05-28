classdef ActionKinematicConstraint
  
  properties (SetAccess=protected)
    name='';
    body_ind
    body_pts 
    pos_min   % trajectory
    pos_max   % trajectory
    tspan     % 1x2 double
    robot
    % contact_state = -1 undefined
    % contact_state = 0 not in contact
    % contact_state = 1 makes contact right at that moment
    % contact_state = 2 breaks contact right at that moment
    % contact_state = 3 in static planar contact
    % contact_state = 4 in static gripping contact
    contact_dist_min % A cell, contact_dist_min can be nonzero only in NOT_IN_CONTACT state
    contact_dist_max % A cell contact_dist_max can be nonzero only in NOT_IN_CONTACT state
    contact_grp
    contact_aff % A cell contains all affordance objects for the same body, can be either a ContactAffordance object, or a RigidBody Object
    num_aff     % the number of affordance
  end
  properties
    contact_state0 % A cell, containing the starting contact state, a row vector, each entry is for one body point
    contact_statei % A cell, containing the contact state in between the starting and ending
    contact_statef % A cell, contarining the ending contact state.
  end
  properties(Constant)
    UNDEFINED_CONTACT = -1;
    NOT_IN_CONTACT = 0;
    MAKE_CONTACT = 1;
    BREAK_CONTACT = 2;
    STATIC_PLANAR_CONTACT = 3;
    STATIC_GRIP_CONTACT = 4;
    COLLISION_AVOIDANCE = 5;
  end
  
  methods
    function obj = ActionKinematicConstraint(robot,body_ind,body_pts,worldpos,tspan,name,contact_state0,contact_statei,contact_statef,contact_aff,contact_distance,contact_grp)
      if nargin < 12
        contact_grp = '';
      end
      obj.robot = robot;
      obj.contact_grp = contact_grp;
      sizecheck(body_ind,1);

      
       
      if(isa(body_ind,'RigidBody'))
        body_ind = robot.findLinkInd(body_ind.linkname);
      end
      obj.body_ind = body_ind;
      
      if(ischar(body_pts)||numel(body_pts)==1)
          body_pts = robot.body(body_ind).getContactPoints(body_pts);
      end
      sizecheck(body_pts,[3 nan]);
      obj.body_pts = body_pts;

      obj = setPosMinMax(obj,worldpos);

      sizecheck(tspan,[1 2]);
      obj.tspan = tspan;


      if (nargin>5)
        typecheck(name,'char');
        obj.name = name;
      end

      if(nargin>6)
        if(nargin<10)
          contact_aff = {ContactAffordance()};
          contact_distance{1}.min = ConstantTrajectory(zeros(1,size(obj.body_pts,2)));
          %contact_distance{1}.max = ConstantTrajectory(inf(1,size(obj.body_pts,2)));
          contact_distance{1}.max = ConstantTrajectory(zeros(1,size(obj.body_pts,2)));
        end
        if(~iscell(contact_state0))
          contact_state0 = {contact_state0};
        end
        if(~iscell(contact_statei))
          contact_statei = {contact_statei};
        end
        if(~iscell(contact_statef))
          contact_statef = {contact_statef};
        end
        obj = setContact(obj,contact_state0,contact_statei,contact_statef,contact_aff,contact_distance);
      else
        contact_state0 = {-ones(1,size(obj.body_pts,2))};
        contact_statei = {-ones(1,size(obj.body_pts,2))};
        contact_statef = {-ones(1,size(obj.body_pts,2))};
        contact_aff = {ContactAffordance()};
        contact_dist{1}.min = ConstantTrajectory(zeros(1,size(obj.body_pts,2)));
        contact_dist{1}.max = ConstantTrajectory(inf(1,size(obj.body_pts,2)));
        obj = setContact(obj,contact_state0,contact_statei,contact_statef,contact_aff,contact_dist);
      end
    end

    function obj = setBodyInd(obj,body)
      switch class(body)
        case 'RigidBody'
          body = obj.robot.findLinkInd(body.linkname);
        case 'char'
          body = obj.robot.findLinkInd(body);
      end
      obj.body_ind = body;
    end

    function body_ind = getBodyInd(obj)
      body_ind = obj.body_ind;
    end
    
    function obj = setTspan(obj,tspan,ind)
      if nargin < 3
        sizecheck(tspan,2);
        obj.tspan = tspan;
      else
        sizecheck(tspan,1);
        rangecheck(ind,1,2);
        obj.tspan(ind) = tspan;
      end
    end

    function obj = setName(obj,name)
      obj.name = name;
    end

    function obj = setContactGrp(obj,contact_grp)
      obj.contact_grp = contact_grp;
    end

    function obj = setPosMinMax(obj,worldpos)
      mi = size(obj.body_pts,2);
      if isstruct(worldpos)
        if ~isfield(worldpos,'min') || ~isfield(worldpos,'max')
          error('if worldpos is a struct, it must have fields .min and .max');
        end
        minpos=[worldpos.min];  maxpos=[worldpos.max];
      else
        minpos=worldpos; maxpos=worldpos;
      end
      if(isa(worldpos,'Trajectory'))
          worldpos_size = size(minpos);
          rows = worldpos_size(1);
          cols = worldpos_size(2);
      else
          [rows,cols]=size(minpos);
      end
      if (rows ~= 3 && rows ~= 6 && rows ~=7) error('worldpos must have 3 or 6 rows'); end
      if (obj.body_ind==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
      if (cols~=mi) error('worldpos must have the same number of elements as bodypos'); end
      sizecheck(maxpos,[rows,mi]);
      if(~isa(worldpos,'Trajectory'))
          minpos(isnan(minpos))=-inf;
          maxpos(isnan(maxpos))=inf;
      end
      if isa(minpos,'Trajectory')
        obj.pos_min = minpos;
      else
        obj.pos_min = ConstantTrajectory(minpos);
      end
      if isa(maxpos,'Trajectory')
        obj.pos_max = maxpos;
      else
        obj.pos_max = ConstantTrajectory(maxpos);
      end
      
      
    end
    
    function ikargs = getIKArguments(obj,t)
        % Return 6 arguments, the body index, the body points, the world
        % position, the contact state, the contact affordances, and the
        % contact distance
      if t<obj.tspan(1) || t>obj.tspan(2)
        ikargs={};
      else
        pos.min = eval(obj.pos_min,t);
        pos.max = eval(obj.pos_max,t);
        contact_state = obj.getContactState(t);
        contact_dist = cell(1,obj.num_aff);
        for i = 1:obj.num_aff
            contact_dist{i} = struct();
            contact_dist{i}.min = eval(obj.contact_dist_min{i},t);
            contact_dist{i}.max = eval(obj.contact_dist_max{i},t);
            if(any(contact_dist{i}.max(contact_state{i} == obj.MAKE_CONTACT|contact_state{i}==obj.BREAK_CONTACT|...
                    contact_state{i} == obj.STATIC_PLANAR_CONTACT| contact_state{i} == obj.STATIC_GRIP_CONTACT)>eps))
                error('The maximum contact distance should be zero for points in contact')
            end
        end
        if (obj.body_ind==0)
          ikargs = {obj.body_ind,pos,contact_state,obj.contact_aff,contact_dist};
        else
          ikargs = {obj.body_ind,obj.body_pts,pos,contact_state,obj.contact_aff,contact_dist};
        end
      end
    end
    
    function contact_state = getContactState(obj,t)
        if(t == obj.tspan(1))
            contact_state = obj.contact_state0;
        elseif(t == obj.tspan(end))
            contact_state = obj.contact_statef;
        elseif(obj.tspan(1)<t&&t<obj.tspan(end))
            contact_state = obj.contact_statei;
        end
        if(~iscell(contact_state))
            error('Contact state should be a cell. If there is only one affordance, then the contact state should be a cell with one entry');
        end
    end
    
    function obj = setContact(obj,contact_state0,contact_statei,contact_statef,contact_aff,contact_distance)
        if(~iscell(contact_state0)||~iscell(contact_statei)||~iscell(contact_statef)||~iscell(contact_aff)||~iscell(contact_distance))
            error('All input arguments to setContact should be cells, each cell has the number of entries equal to the number of contact affordance');
        end
        obj.num_aff = length(contact_aff);
      obj.contact_state0 = contact_state0;
      obj.contact_statei = contact_statei;
      obj.contact_statef = contact_statef;
      obj.contact_aff = cell(1,obj.num_aff);
      contact_aff_names = cell(1,obj.num_aff);
      % check if the contact state if a valid combination
      valid_contact_comb = [obj.UNDEFINED_CONTACT obj.UNDEFINED_CONTACT obj.UNDEFINED_CONTACT;...
          obj.NOT_IN_CONTACT obj.NOT_IN_CONTACT obj.NOT_IN_CONTACT;...
          obj.NOT_IN_CONTACT obj.NOT_IN_CONTACT obj.MAKE_CONTACT;...
          obj.MAKE_CONTACT obj.STATIC_PLANAR_CONTACT obj.STATIC_PLANAR_CONTACT;...
          obj.MAKE_CONTACT obj.STATIC_PLANAR_CONTACT obj.BREAK_CONTACT;...
          obj.MAKE_CONTACT obj.STATIC_GRIP_CONTACT obj.STATIC_GRIP_CONTACT;...
          obj.MAKE_CONTACT obj.STATIC_GRIP_CONTACT obj.BREAK_CONTACT;...
          obj.BREAK_CONTACT obj.NOT_IN_CONTACT obj.NOT_IN_CONTACT;...
          obj.BREAK_CONTACT obj.NOT_IN_CONTACT obj.MAKE_CONTACT;...
          obj.STATIC_PLANAR_CONTACT obj.STATIC_PLANAR_CONTACT obj.STATIC_PLANAR_CONTACT;...
          obj.STATIC_PLANAR_CONTACT obj.STATIC_PLANAR_CONTACT obj.BREAK_CONTACT;...
          obj.STATIC_PLANAR_CONTACT obj.BREAK_CONTACT obj.NOT_IN_CONTACT;...
          obj.STATIC_GRIP_CONTACT obj.STATIC_GRIP_CONTACT obj.STATIC_GRIP_CONTACT;...
          obj.STATIC_GRIP_CONTACT obj.STATIC_GRIP_CONTACT obj.BREAK_CONTACT;...
          obj.STATIC_GRIP_CONTACT obj.BREAK_CONTACT obj.NOT_IN_CONTACT;...
          obj.UNDEFINED_CONTACT obj.COLLISION_AVOIDANCE obj.UNDEFINED_CONTACT;...
          obj.UNDEFINED_CONTACT obj.COLLISION_AVOIDANCE obj.COLLISION_AVOIDANCE;...
          obj.COLLISION_AVOIDANCE obj.COLLISION_AVOIDANCE obj.UNDEFINED_CONTACT;...
          obj.COLLISION_AVOIDANCE obj.COLLISION_AVOIDANCE obj.COLLISION_AVOIDANCE];
      n_pts = size(obj.body_pts,2);
      for i = 1:obj.num_aff
          sizecheck(obj.contact_state0{i},[1,n_pts]);
          sizecheck(obj.contact_statei{i},[1,n_pts]);
          sizecheck(obj.contact_statef{i},[1,n_pts]);
          valid_contact_flag = ismember([obj.contact_state0{i};obj.contact_statei{i};obj.contact_statef{i}]',valid_contact_comb,'rows');
          if(~all(valid_contact_flag))
              error('The contact state for the %d th point is not valid',find(~valid_contact_flag));
          end
          % allow contact_aff{i} being a rigid body, as the latter is the
          % input to the pairwiseContactConstraint function
          if(isa(contact_aff{i},'ContactAffordance')||isa(contact_aff{i},'RigidBody'))
            obj.contact_aff{i} = contact_aff{i};
          if(isa(contact_aff{i},'ContactAffordance'))
            obj.contact_aff{i} = contact_aff{i};
            contact_aff_names{i} = contact_aff{i}.name;
            elseif(isa(contact_aff{i},'RigidBody'))
                contact_aff_names{i} = contact_aff{i}.linkname;
            end
            if(i>1)
                if(any(cellfun(@(x) strcmp(x,contact_aff_names{i}),contact_aff_names(1:i-1))))
                    error('The contact affordances should not be the same, check the name of the affordance')
                end
            end
            if(any(obj.contact_statei{i}==obj.COLLISION_AVOIDANCE))
                sizecheck(obj.contact_statei,[1,1]); % For collision avoidance, 
                if(any(obj.body_pts ~= [0;0;0]))
                    error('For collision avoidance, the body points must be [0;0;0]');
                end
            end
          else
              error('contact_aff must be either a ContactAffordance object or a RigidBody object');
          end
          if(isstruct(contact_distance{i}))
              if(~isfield(contact_distance{i},'min')||~isfield(contact_distance{i},'max'))
                  error('If contact distance is a struct, it should have min and max field');
              end
              min_contact_dist = contact_distance{i}.min;
              max_contact_dist = contact_distance{i}.max;
          else
              min_contact_dist = contact_distance{i};
              max_contact_dist = contact_distance{i};
          end
          if(isa(min_contact_dist,'Trajectory'))
              contact_dist_size = size(min_contact_dist);
              rows = contact_dist_size(1);
              cols = contact_dist_size(2);
          else
              [rows,cols] = size(min_contact_dist);
          end
          mi = size(obj.body_pts,2);
          if (rows~=1) error('The contact distance can have only 1 row');end
          if (cols~=mi) error('The contact distance should have the same number of columns as the body pts'); end
          sizecheck(max_contact_dist,[1,mi]);
          if(~isa(min_contact_dist,'Trajectory'))
              min_contact_dist(isnan(min_contact_dist)) = 0;
              max_contact_dist(isnan(max_contact_dist)) = inf;
          end
          if(isa(min_contact_dist,'Trajectory'))
              obj.contact_dist_min{i} = min_contact_dist;
          else
              obj.contact_dist_min{i} = ConstantTrajectory(min_contact_dist);
          end
          if(isa(max_contact_dist,'Trajectory'))
              obj.contact_dist_max{i} = max_contact_dist;
          else
            obj.contact_dist_max{i} = ConstantTrajectory(max_contact_dist);
          end
      end


        
%       end
    end
    
    function obj = setContactState(obj,contact_state,time_str)
      rangecheck(contact_state,-1,4);
      if nargin < 3
        obj.contact_state0 = {repmat(contact_state,1,size(obj.body_pts,2))};
        obj.contact_statei = {repmat(contact_state,1,size(obj.body_pts,2))};
        obj.contact_statef = {repmat(contact_state,1,size(obj.body_pts,2))};      
      else
        switch time_str
          case 't0'
            obj.contact_state0 = {repmat(contact_state,1,size(obj.body_pts,2))};
          case 'ti'
            obj.contact_statei = {repmat(contact_state,1,size(obj.body_pts,2))};
          case 'tf'
            obj.contact_statef = {repmat(contact_state,1,size(obj.body_pts,2))};      
          otherwise
            error('The input argument time_str must be either ''t0'', ''ti'', or, ''tf''');
        end
      end
    end
  end
  
%   methods (Static=true)
%     function obj = onGroundConstraint(robot,body_ind,body_pts,tspan,name)
%       pos.min = repmat([nan;nan;0],1,size(body_pts,2));
%       pos.max = repmat([nan;nan;0],1,size(body_pts,2));
%       n_pts = size(body_pts,2);
%       contact_state0 = ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,n_pts);
%       contact_statei = ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,n_pts);
%       contact_statef = ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,n_pts);
%       contact_dist = zeros(1,n_pts);
%       obj = ActionKinematicConstraint(robot,body_ind,body_pts,pos,tspan,name);
%     end
%     
%     function obj = groundConstraint(robot,body_ind,body_pts,tspan,name)
%         pos.min = repmat([nan;nan;0],1,size(body_pts,2));
%         pos.max = repmat([nan;nan;nan],1,size(body_pts,2));
%         obj = ActionKinematicConstraint(robot,body_ind,body_pts,pos,tspan,name);
%     end
%   end
end

classdef ActionSequence
  % structure which lists a series of kinematic (and possibly dynamic)
  % objectives and constraints for a robot
  
  properties
    %% Objectives
    
    %% Constraints
    kincons = {};  % a cell array of ActionKinematicConstraints
    kincon_name = {};
    tspan=[inf, -inf];
    key_time_samples = [];
    contact_time = []; % This records the time when making or breaking contact
    make_contact_time = [];
    break_contact_time = [];
  end
  
  methods
    
    function obj = ActionSequence(kc_array)
      if nargin > 0
        typecheck(kc_array,'ActionKinematicConstraint');
        for kc = reshape(kc_array,1,[])
          obj = addKinematicConstraint(obj,kc);
        end
      end
    end

    function obj=addKinematicConstraint(obj,kc)
      typecheck(kc,'ActionKinematicConstraint');
      obj.kincons = [obj.kincons,{kc}];
%       if(strcmp(kc.name,obj.kincon_name))
%           error('kinematic constraint name exists, change it to a new name before adding the constraint to the sequence');
%       end
      obj.kincon_name = [obj.kincon_name,{kc.name}];
      obj.tspan(1) = min(obj.tspan(1),kc.tspan(1));
      obj.tspan(2) = max(obj.tspan(2),kc.tspan(2));
      obj.key_time_samples = [obj.key_time_samples, kc.tspan(1) kc.tspan(2)];
      obj.key_time_samples = unique(obj.key_time_samples);
      ikargs1 = kc.getIKArguments(kc.tspan(1));
      ikargs2 = kc.getIKArguments(kc.tspan(2));
      if(ikargs1{1} ~=0)
          k = 1;
          while(k<=length(ikargs1{3}.contact_state))
              if(any(ikargs1{3}.contact_state{k}==ActionKinematicConstraint.MAKE_CONTACT))
                  obj.make_contact_time = [obj.make_contact_time kc.tspan(1)];
              end
              if(any(ikargs2{3}.contact_state{k}==ActionKinematicConstraint.BREAK_CONTACT))
                  obj.break_contact_time = [obj.break_contact_time kc.tspan(2)];
              end
              k = k+1;
          end
          obj.contact_time = unique([obj.make_contact_time obj.break_contact_time]);
      end
    end
    
    function obj = addStaticContactConstraint(obj,robot,q,start_time)
        % This function loops inside the kincons, searches which kincon has
        % contact_state == 1 at the start time, and constrain those points
        % to be in the same position until the contact_state = 2 (when it breaks contact)

        kinsol = doKinematics(robot,q);
        for i = 1:length(obj.kincons)
            if(start_time == obj.kincons{i}.tspan(1))
                body_pts_ind = obj.kincons{i}.contact_state0{1} == 1&obj.kincons{i}.contact_statef{1} == 2;
                %if(~isempty(body_pts_ind))
                if(any(body_pts_ind(:)))
                    body_ind = obj.kincons{i}.body_ind;
                    body_pts = obj.kincons{i}.body_pts(:,body_pts_ind);
                    worldpos = forwardKin(robot,kinsol,body_ind,body_pts,0);
                    name = [robot.getLinkName(body_ind),' static contact from ',num2str(obj.kincons{i}.tspan(1)),' to ',num2str(obj.kincons{i}.tspan(end))];
                    kc = ActionKinematicConstraint(robot,body_ind,body_pts,worldpos,obj.kincons{i}.tspan,name,obj.kincons{i}.contact_state0{1}(body_pts_ind),3*ones(size(body_pts_ind)),obj.kincons{i}.contact_statef{1}(body_pts_ind));
                    obj = obj.addKinematicConstraint(kc);
                end
            end
        end
    end
    
    function obj = deleteKinematicConstraint(obj,kc_name)
        kc_ind = (~strcmp(kc_name,obj.kincon_name));
        obj.kincons = obj.kincons(kc_ind);
        obj.kincon_name = obj.kincon_name(kc_ind);
        key_time_samples = [];
        for i = 1:length(obj.kincons)
            key_time_samples = [key_time_samples obj.kincons{i}.tspan];
        end
        obj.key_time_samples = unique(key_time_samples);
        obj.tspan = [obj.key_time_samples(1) obj.key_time_samples(end)];
    end
    
    function ikargs = getIKArguments(obj,t)
      ikargs={};
      % Here the contact points on the same body should be aggregated
      % together
      bodys = [];
      bodys_ikargs = {};
      collision_bodys = [];
      collision_ikargs = {}; % We do not aggregate the ikargs for collision to the ikargs of contact, since they are not for contact points, but for the body.
      for i=1:length(obj.kincons)
        if(isa(obj.kincons{i},'CollisionAvoidanceConstraint'))
          kin_col_ikargs = obj.kincons{i}.getIKArguments(t);
          if(~isempty(kin_col_ikargs))
            collision_state = kin_col_ikargs{3}.contact_state{1};
            if(collision_state == ActionKinematicConstraint.UNDEFINED_CONTACT)
              if(t == obj.tspan(1)|| t == obj.tspan(end))
                collision_state = ActionKinematicConstraint.COLLISION_AVOIDANCE;
              else
                continue;
              end
            end
            kin_col_ikargs{3}.contact_state{1} = collision_state;
            body_ind = kin_col_ikargs{1};
            bodys_ind = find(body_ind == collision_bodys,1);
            if(isempty(bodys_ind))
              collision_bodys = [collision_bodys body_ind];
              collision_ikargs = [collision_ikargs {kin_col_ikargs}];
            else
              collision_ikargs{bodys_ind}{3}.contact_state = [collision_ikargs{bodys_ind}{3}.contact_state {collision_state}];
              collision_ikargs{bodys_ind}{3}.contact_affs = [collision_ikargs{bodys_ind}{3}.contact_affs kin_col_ikargs{3}.contact_affs];
              collision_ikargs{bodys_ind}{3}.contact_dist = [collision_ikargs{bodys_ind}{3}.contact_dist kin_col_ikargs{3}.contact_dist];
            end
          end
        else
          kincon_ikargs = obj.kincons{i}.getIKArguments(t);
          if(~isempty(kincon_ikargs))
            body_ind = kincon_ikargs{1}; % This is the index in the robot
            bodys_ind = find(body_ind == bodys,1); % This is the index in the ik arguments
            if(body_ind==0)
              worldpos_ind = 2;
            else
              worldpos_ind = 3;
            end
            if(~isstruct(kincon_ikargs{worldpos_ind}))
              worldpos = struct();
              worldpos.max = kincon_ikargs{worldpos_ind};
              worldpos.min = kincon_ikargs{worldpos_ind};
              kincon_ikargs{worldpos_ind} = worldpos;
            end
            if(isempty(bodys_ind))
              bodys = [bodys body_ind];
              bodys_ikargs = [bodys_ikargs,{kincon_ikargs}];
            else
              if(body_ind~=0)
                bodys_pts = [bodys_ikargs{bodys_ind}{2} kincon_ikargs{2}];
                worldpos = struct();
                if(size(bodys_ikargs{bodys_ind}{3}.max,1) == 6&&size(kincon_ikargs{3}.max,1)==3)
                  kincon_ikargs{3}.max = [kincon_ikargs{3}.max;inf(3,size(kincon_ikargs{3}.max,2))];
                  kincon_ikargs{3}.min = [kincon_ikargs{3}.min;-inf(3,size(kincon_ikargs{3}.min,2))];
                elseif(size(bodys_ikargs{bodys_ind}{3}.max,1) == 3&&size(kincon_ikargs{3}.max,1)==6)
                  bodys_ikargs{bodys_ind}{3}.max = [bodys_ikargs{bodys_ind}{3}.max inf(3,size(bodys_ikargs{bodys_ind}{3}.max,2))];
                  bodys_ikargs{bodys_ind}{3}.min = [bodys_ikargs{bodys_ind}{3}.min -inf(3,size(bodys_ikargs{bodys_ind}{3}.min,2))];
                elseif(size(bodys_ikargs{bodys_ind}{3}.max,1) == 7&&size(kincon_ikargs{3}.max,1)==3)
                  kincon_ikargs{3}.max = [kincon_ikargs{3}.max;ones(4,size(kincon_ikargs{3}.max,2))];
                  kincon_ikargs{3}.min = [kincon_ikargs{3}.min;-ones(4,size(kincon_ikargs{3}.min,2))];
                elseif(size(bodys_ikargs{bodys_ind}{3}.max,1) == 3&&size(kincon_ikargs{3}.max,1)==7)
                  bodys_ikargs{bodys_ind}{3}.max = [bodys_ikargs{bodys_ind}{3}.max ones(4,size(bodys_ikargs{bodys_ind}{3}.max,2))];
                  bodys_ikargs{bodys_ind}{3}.min = [bodys_ikargs{bodys_ind}{3}.min -ones(4,size(bodys_ikargs{bodys_ind}{3}.min,2))];
                elseif((size(bodys_ikargs{bodys_ind}{3}.max,1) == 6&&size(kincon_ikargs{3}.max,1)==7)...
                    ||(size(bodys_ikargs{bodys_ind}{3}.max,1) == 7&&size(kincon_ikargs{3}.max,1)==6))
                  error('Currently I cannot support that both quaternion and Euler angles are used for the same contact point at the same time');
                end
                worldpos.max = [bodys_ikargs{bodys_ind}{3}.max kincon_ikargs{3}.max];
                worldpos.min = [bodys_ikargs{bodys_ind}{3}.min kincon_ikargs{3}.min];
                [bodys_unique_pts,bodys_unique_pts_ind,body_pts_ind] = unique(bodys_pts','rows');
                bodys_unique_pts = bodys_unique_pts';
                num_bodys_unique_pts = size(bodys_unique_pts,2);
                worldpos_unique = struct();
                worldpos_unique.max = inf(size(worldpos.max,1),num_bodys_unique_pts);
                worldpos_unique.min = -inf(size(worldpos.min,1),num_bodys_unique_pts);
                for j = 1:size(worldpos.max,2)
                  worldpos_unique.max(:,body_pts_ind(j)) = min([worldpos_unique.max(:,body_pts_ind(j)) worldpos.max(:,j)],[],2);
                  worldpos_unique.min(:,body_pts_ind(j)) = max([worldpos_unique.min(:,body_pts_ind(j)) worldpos.min(:,j)],[],2);
                  worldpos_unique.max(:,body_pts_ind(j)) = max([worldpos_unique.max(:,body_pts_ind(j)) worldpos_unique.min(:,body_pts_ind(j))],[],2);
                end
                
                % Should consider aggregating the contact constraints
                % also, to find out the minimal set of distance
                org_body_pts_ind = body_pts_ind(1:size(bodys_ikargs{bodys_ind}{2},2));
                kincon_pts_ind = body_pts_ind(size(bodys_ikargs{bodys_ind}{2},2)+1:end);
                org_num_contact_aff = length(bodys_ikargs{bodys_ind}{3}.contact_affs);
                kincon_num_contact_aff = length(kincon_ikargs{3}.contact_affs);
                contact_state = cell(1,org_num_contact_aff+kincon_num_contact_aff);
                contact_affs = cell(1,org_num_contact_aff+kincon_num_contact_aff);
                contact_dist = cell(1,org_num_contact_aff+kincon_num_contact_aff);
                for k = 1:org_num_contact_aff
                  contact_state{k} = ActionKinematicConstraint.UNDEFINED_CONTACT*ones(1,num_bodys_unique_pts);
                  contact_state{k}(org_body_pts_ind) = bodys_ikargs{bodys_ind}{3}.contact_state{k};
                  contact_dist{k} = struct();
                  contact_dist{k}.max = inf(1,num_bodys_unique_pts);
                  contact_dist{k}.min = zeros(1,num_bodys_unique_pts);
                  contact_dist{k}.max(org_body_pts_ind) = bodys_ikargs{bodys_ind}{3}.contact_dist{k}.max;
                  contact_dist{k}.min(org_body_pts_ind) = bodys_ikargs{bodys_ind}{3}.contact_dist{k}.min;
                  contact_affs{k} = bodys_ikargs{bodys_ind}{3}.contact_affs{k};
                end
                for k = 1:kincon_num_contact_aff
                  contact_state{org_num_contact_aff+k} = ActionKinematicConstraint.UNDEFINED_CONTACT*ones(1,num_bodys_unique_pts);
                  contact_state{org_num_contact_aff+k}(kincon_pts_ind) = kincon_ikargs{3}.contact_state{k};
                  contact_dist{org_num_contact_aff+k} = struct();
                  contact_dist{org_num_contact_aff+k}.max = inf(1,num_bodys_unique_pts);
                  contact_dist{org_num_contact_aff+k}.min = zeros(1,num_bodys_unique_pts);
                  contact_dist{org_num_contact_aff+k}.max(kincon_pts_ind) = kincon_ikargs{3}.contact_dist{k}.max;
                  contact_dist{org_num_contact_aff+k}.min(kincon_pts_ind) = kincon_ikargs{3}.contact_dist{k}.min;
                  contact_affs{org_num_contact_aff+k} = kincon_ikargs{3}.contact_affs{k};
                end
                worldpos_unique.contact_state = contact_state;
                worldpos_unique.contact_affs = contact_affs;
                worldpos_unique.contact_dist = contact_dist;
                bodys_ikargs{bodys_ind} = {body_ind,bodys_unique_pts,worldpos_unique};
              else
                worldpos_unique.max = min([bodys_ikargs{bodys_ind}{2}.max kincon_ikargs{2}.max],[],2);
                worldpos_unique.min = max([bodys_ikargs{bodys_ind}{2}.min kincon_ikargs{2}.min],[],2);
                worldpos_unique.max = max([worldpos_unique.max worldpos_unique.min],[],2);
                worldpos_unique.contact_state = ActionKinematicConstraint.UNDEFINED_CONTACT;
                worldpos_unique.contact_affs = ContactAffordance;
                worldpos_unique.contact_dist = struct('min',0,'max',inf);
                bodys_ikargs{bodys_ind} = {body_ind,worldpos_unique};
              end
            end
          end
        end
      end
      ikargs = [bodys_ikargs{:} collision_ikargs{:}];
    end

    function contact_states = getContactStates(obj,t)
      contact_states = cellfun(@(kc)getContactState(kc,t),obj.kincons,'UniformOutput',false);
    end

    function obj = setContactStates(obj,kc_ind,contact_state,time_str)
      obj.kincons(kc_ind) = obj.kincons(kc_ind).setContactState(contact_state,time_str);
    end

    function kincons = getKinematicConstraints(obj)
      kincons = obj.kincons;
    end
    
    function obj = generateImplicitConstraints(obj, robot, q0, options)
      NB = robot.getNumBodies;
      tspan = [0 obj.tspan(1)];
      if isfield(options,'initial_contact_groups')
          initial_contact_link_inds = zeros(size(options.initial_contact_groups.linknames))
          for i = 1:length(options.initial_contact_groups.linknames)
              initial_contact_link_inds(i) = findLinkInd(robot,options.initial_contact_groups.linknames{i});
          end
      end
      for i=2:NB
        ptsA = []; ptsB = [];
        kinsol = doKinematics(robot,q0);
        if isfield(options,'initial_contact_groups')
           
          link_ind = find(i == initial_contact_link_inds);
          if ~isempty(link_ind)
            for ind = reshape(link_ind,1,[])
              ptsB_new = getContactPoints(robot.getBody(i),options.initial_contact_groups.groupnames{ind});
              ptsB = [ptsB, ptsB_new];
              ptsA = [ptsA, forwardKin(robot,kinsol,i,ptsB_new)];
            end
          end
        else
          [ptsA,ptsB] = pairwiseContactTest(robot,kinsol,1,i);
        end
        if ~isempty(ptsA)
          kc = createContactConstraint(robot,1,ptsA,i,ptsB,tspan);
          for kc = reshape(kc,1,[])
            obj = addKinematicConstraint(obj,kc);
          end
        end
      end
      
      function kc = createContactConstraint(robot,body_indA,ptsA,body_indB,ptsB,tspan);
        % dummy implementation - assumes that bodyA is the world link !!!
        % Creates two `ActionKinematicConstraint`
        %   z equality constraint
        %   xy bounding box
        
        n_pts = size(ptsA,2);
        name_z = sprintf('z_%s_in_contact_with_%s_from_%3.1f_to_%3.1f', ...
          robot.getLinkName(body_indB), robot.getLinkName(body_indA), tspan);
        name_xy = sprintf('xy_%s_in_contact_with_%s_from_%3.1f_to_%3.1f', ...
          robot.getLinkName(body_indB), robot.getLinkName(body_indA), tspan);
        worldpos_z.min = [-Inf(2,n_pts); ptsA(3,:)];
        worldpos_z.max = [Inf(2,size(ptsA,2)); ptsA(3,:)];
        worldpos_xy.min = repmat([min(ptsA(1:2,:),[],2); -Inf],1,n_pts);
        worldpos_xy.max = repmat([max(ptsA(1:2,:),[],2); Inf],1,n_pts);
        kc = [ActionKinematicConstraint(robot,body_indB,ptsB,worldpos_z,tspan, name_z, ...
          {ActionKinematicConstraint.MAKE_CONTACT*ones(1,n_pts)}, ...
          {ActionKinematicConstraint.STATIC_PLANAR_CONTACT*ones(1,n_pts)}, ...
          {ActionKinematicConstraint.BREAK_CONTACT*ones(1,n_pts)}, {body_indA}, {zeros(1,n_pts)}); ...
          ActionKinematicConstraint(robot,body_indB,ptsB,worldpos_xy,tspan, name_xy, ...
          {ActionKinematicConstraint.NOT_IN_CONTACT*ones(1,n_pts)}, ...
          {ActionKinematicConstraint.NOT_IN_CONTACT*ones(1,n_pts)}, ...
          {ActionKinematicConstraint.NOT_IN_CONTACT*ones(1,n_pts)}, {body_indA}, {zeros(1,n_pts)})];
        kc = kc(1);
      end
      
    end

    
  end
  
end

% NORELEASE
  

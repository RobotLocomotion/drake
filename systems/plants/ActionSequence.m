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
          while(k<=length(ikargs1{4}))
              if(any(ikargs1{4}{k}==ActionKinematicConstraint.MAKE_CONTACT))
                  obj.make_contact_time = [obj.make_contact_time kc.tspan(1)];
              end
              if(any(ikargs2{4}{k}==ActionKinematicConstraint.BREAK_CONTACT))
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
                    name = [robot.body(body_ind).linkname,' static contact from ',num2str(obj.kincons{i}.tspan(1)),' to ',num2str(obj.kincons{i}.tspan(end))];
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
      for i=1:length(obj.kincons)
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
                    org_num_contact_aff = length(bodys_ikargs{bodys_ind}{5});
                    kincon_num_contact_aff = length(kincon_ikargs{5});
                    contact_state = cell(1,org_num_contact_aff+kincon_num_contact_aff);
                    contact_aff = cell(1,org_num_contact_aff+kincon_num_contact_aff);
                    contact_dist = cell(1,org_num_contact_aff+kincon_num_contact_aff);
                    for k = 1:org_num_contact_aff
                        contact_state{k} = ActionKinematicConstraint.UNDEFINED_CONTACT*ones(1,num_bodys_unique_pts);
                        contact_state{k}(org_body_pts_ind) = bodys_ikargs{bodys_ind}{4}{k};
                        contact_dist{k} = struct();
                        contact_dist{k}.max = inf(1,num_bodys_unique_pts);
                        contact_dist{k}.min = zeros(1,num_bodys_unique_pts);
                        contact_dist{k}.max(org_body_pts_ind) = bodys_ikargs{bodys_ind}{6}{k}.max;
                        contact_dist{k}.min(org_body_pts_ind) = bodys_ikargs{bodys_ind}{6}{k}.min;
                        contact_aff{k} = bodys_ikargs{bodys_ind}{5}{k};
                    end
                    for k = 1:kincon_num_contact_aff
                        contact_state{org_num_contact_aff+k} = ActionKinematicConstraint.UNDEFINED_CONTACT*ones(1,num_bodys_unique_pts);
                        contact_state{org_num_contact_aff+k}(kincon_pts_ind) = kincon_ikargs{4}{k};
                        contact_dist{org_num_contact_aff+k} = struct();
                        contact_dist{org_num_contact_aff+k}.max = inf(1,num_bodys_unique_pts);
                        contact_dist{org_num_contact_aff+k}.min = zeros(1,num_bodys_unique_pts);
                        contact_dist{org_num_contact_aff+k}.max(kincon_pts_ind) = kincon_ikargs{6}{k}.max;
                        contact_dist{org_num_contact_aff+k}.min(kincon_pts_ind) = kincon_ikargs{6}{k}.min;
                        contact_aff{org_num_contact_aff+k} = kincon_ikargs{5}{k};
                    end
                    bodys_ikargs{bodys_ind} = {body_ind,bodys_unique_pts,worldpos_unique,contact_state,contact_aff,contact_dist};
                  else
                      worldpos_unique.max = min([bodys_ikargs{body_ind}{2}.max kincon_ikargs{2}.max],[],2);
                      worldpos_unique.min = max([bodys_ikargs{body_ind}{2}.min kincon_ikargs{2}.min],[],2);
                      worldpos_unique.max = max([worldpos_unique.max worldpos_unique.min]);
                      bodys_ikargs{bodys_ind} = {body_ind,worldpos_unique,-1,ContactAffordance,struct('min',0,'max',inf)};
                  end
              end
          end
      end
      ikargs = [bodys_ikargs{:}];
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
  end
  
end

% NORELEASE
  

classdef ActionSequence
  % structure which lists a series of kinematic (and possibly dynamic)
  % objectives and constraints for a robot
  
  properties (Access=protected)
    %% Objectives
    
    %% Constraints
    kincon = {};  % a cell array of ActionKinematicConstraints
    kincon_name = {};
  end
  properties
    tspan=[inf, -inf];
    key_time_samples = []
  end
  
  methods
    
    function obj=addKinematicConstraint(obj,kc)
      typecheck(kc,'ActionKinematicConstraint');
      obj.kincon = [obj.kincon,{kc}];
%       if(strcmp(kc.name,obj.kincon_name))
%           error('kinematic constraint name exists, change it to a new name before adding the constraint to the sequence');
%       end
      obj.kincon_name = [obj.kincon_name,{kc.name}];
      obj.tspan(1) = min(obj.tspan(1),kc.tspan(1));
      obj.tspan(2) = max(obj.tspan(2),kc.tspan(2));
      obj.key_time_samples = [obj.key_time_samples, kc.tspan(1) kc.tspan(2)];
      obj.key_time_samples = unique(obj.key_time_samples);
    end
    
    function obj = deleteKinematicConstraint(obj,kc_name)
        kc_ind = (~strcmp(kc_name,obj.kincon_name));
        obj.kincon = obj.kincon(kc_ind);
        obj.kincon_name = obj.kincon_name(kc_ind);
        key_time_samples = [];
        for i = 1:length(obj.kincon)
            key_time_samples = [key_time_samples obj.kincon{i}.tspan];
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
      for i=1:length(obj.kincon)
          kincon_ikargs = obj.kincon{i}.getIKArguments(t);
          if(~isempty(kincon_ikargs))
              if(~isstruct(kincon_ikargs{end}))
                  worldpos = struct();
                  worldpos.max = kincon_ikargs{end};
                  worldpos.min = kincon_ikargs{end};
                  kincon_ikargs{end} = worldpos;
              end
              body_ind = kincon_ikargs{1}; % This is the index in the robot
              bodys_ind = find(body_ind == bodys,1); % This is the index in the ik arguments
              if(isempty(bodys_ind))
                  bodys = [bodys body_ind];
                  bodys_ikargs = [bodys_ikargs,{kincon_ikargs}];
              else
                  if(body_ind~=0)
                    bodys_pts = [bodys_ikargs{bodys_ind}{2} kincon_ikargs{2}];
                    worldpos = struct();
                    if(size(bodys_ikargs{bodys_ind}{3}.max,1) == 6&&size(kincon_ikargs{3}.max,1)==3)
                        kincon_ikargs{2} = [kincon_ikargs{2};zeros(3,size(kincon_ikargs{2},2))];
                        kincon_ikargs{3}.max = [kincon_ikargs{3}.max;inf(3,size(kincon_ikargs{3}.max,2))];
                        kincon_ikargs{3}.min = [kincon_ikargs{3}.min;-inf(3,size(kincon_ikargs{3}.min,2))];
                    elseif(size(bodys_ikargs{bodys_ind}{3}.max,1) == 3&&size(kincon_ikargs{3}.max,1)==6)
                        bodys_ikargs{bodys_ind}{2} = [bodys_ikargs{bodys_ind}{2};zeros(3,size(bodys_ikargs{bodys_ind}{2},2))];
                        bodys_ikargs{bodys_ind}{3}.max = [bodys_ikargs{bodys_ind}{3}.max inf(3,size(bodys_ikargs{bodys_ind}{3}.max,2))];
                        bodys_ikargs{bodys_ind}{3}.min = [bodys_ikargs{bodys_ind}{3}.min -inf(3,size(bodys_ikargs{bodys_ind}{3}.min,2))];
                    end
                    worldpos.max = [bodys_ikargs{bodys_ind}{3}.max kincon_ikargs{3}.max];
                    worldpos.min = [bodys_ikargs{bodys_ind}{3}.min kincon_ikargs{3}.min];
                    [bodys_unique_pts,bodys_unique_pts_ind,body_pts_ind] = unique(bodys_pts','rows');
                    bodys_unique_pts = bodys_unique_pts';
                    worldpos_unique = struct();
                    worldpos_unique.max = inf(size(worldpos.max,1),size(bodys_unique_pts,2));
                    worldpos_unique.min = -inf(size(worldpos.min,1),size(bodys_unique_pts,2));
                    for j = 1:size(worldpos.max,2)
                        worldpos_unique.max(:,body_pts_ind(j)) = min([worldpos_unique.max(:,body_pts_ind(j)) worldpos.max(:,j)],[],2);
                        worldpos_unique.min(:,body_pts_ind(j)) = max([worldpos_unique.min(:,body_pts_ind(j)) worldpos.min(:,j)],[],2);
                    end
                    bodys_ikargs{bodys_ind} = {body_ind,bodys_unique_pts,worldpos_unique};
                  else
                      worldpos_unique.max = min([bodys_ikargs{body_ind}{2}.max kincon_ikargs{2}.max],[],2);
                      worldpos_unique.max = max([bodys_ikargs{body_ind}{2}.min kincon_ikargs{2}.min],[],2);
                      bodys_ikargs{bodys_ind} = {body_ind,worldpos_unique};
                  end
              end
          end
      end
      ikargs = [bodys_ikargs{:}];
    end
  end
  
end

% NORELEASE
  
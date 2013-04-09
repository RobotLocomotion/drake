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
    % contact_state = 1 make contact right at that moment
    % contact_state = 2 in static contact
    % contact_state = 3 breaks contact right at that moment
    contact_state0 % The starting contact state, a column vector, each entry is for one body point
    contact_statef % The ending contact state.
  end
  
  methods
    function obj = ActionKinematicConstraint(robot,body_ind,body_pts,worldpos,tspan,name,contact_state0,contact_statef)
      obj.robot = robot;
      sizecheck(body_ind,1);
      if(isa(body_ind,'RigidBody'))
          body_ind = robot.findLinkInd(body_ind.linkname);
      end
      obj.body_ind = body_ind;
      sizecheck(body_pts,[3 nan]);
      obj.body_pts = body_pts;
      
      mi = size(body_pts,2);
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
      if (body_ind==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
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
      
      sizecheck(tspan,[1 2]);
      obj.tspan = tspan;
      
      
      if (nargin>4)
        typecheck(name,'char');
        obj.name = name;
      end
      if(nargin<7)
          obj.contact_state0 = -ones(size(body_pts,2),1);
          obj.contact_statef = -ones(size(body_pts,2),1);
      else
          obj.contact_state0 = contact_state0;
          obj.contact_statef = contact_statef;
      end
    end
    
    function ikargs = getIKArguments(obj,t)
      if t<obj.tspan(1) || t>obj.tspan(2)
        ikargs={};
      else
        pos.min = eval(obj.pos_min,t);
        pos.max = eval(obj.pos_max,t);
        if (obj.body_ind==0)
          ikargs = {obj.body_ind,pos};
        else
          ikargs = {obj.body_ind,obj.body_pts,pos};
        end
      end
    end
    
    function contact_state = getContactState(obj,t)
        contact_state = zeros(size(obj.body_pts,2),1);
        if(t==obj.tspan(1))
            contact_state = obj.contact_state0;
        elseif(t == obj.tspan(end))
            contact_state = obj.contact_statef;
        elseif(t<obj.tspan(end)&&t>obj.tspan(1))
            for j = 1:size(obj.body_pts,2)
                if(obj.contact_state0(j) == -1&&obj.contact_statef(j)==-1)
                    contact_state(j) = -1;
                elseif(obj.contact_state0(j) == 0&&obj.contact_statef(j)==0)
                    contact_state(j) = 0;
                elseif(obj.contact_state0(j) == 2&&obj.contact_statef(j)==2)
                    contact_state(j) = 2;
                elseif(obj.contact_state0(j)==0&&obj.contact_statef(j)==1)
                    contact_state(j) = 0;
                elseif(obj.contact_state0(j)==1&&(obj.contact_statef(j) == 2||obj.contact_statef(j) == 3))
                    contact_state(j) = 2;
                elseif(obj.contact_state0(j) == 2&&obj.contact_statef(j) == 3)
                    contact_state(j) = 2;
                elseif(obj.contact_state0(j) == 3&&(obj.contact_statef(j) == 0||obj.contact_statef(j) == 1))
                    contact_state(j) = 0;
                else
                    error('The contact states are not the valid combination at the two ends of the kinematic constraint');
                end
            end
        else
            error('query time outside of time span of the kinematic constraint')
        end
    end
  end
  
  methods (Static=true)
    function obj = onGroundConstraint(robot,body_ind,body_pts,tspan,name)
      pos.min = repmat([nan;nan;0],1,size(body_pts,2));
      pos.max = repmat([nan;nan;0],1,size(body_pts,2));
      obj = ActionKinematicConstraint(robot,body_ind,body_pts,pos,tspan,name);
    end
    
    function obj = groundConstraint(robot,body_ind,body_pts,tspan,name)
        pos.min = repmat([nan;nan;0],1,size(body_pts,2));
        pos.max = repmat([nan;nan;nan],1,size(body_pts,2));
        obj = ActionKinematicConstraint(robot,body_ind,body_pts,pos,tspan,name);
    end
  end
end
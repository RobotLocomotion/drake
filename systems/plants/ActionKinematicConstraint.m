classdef ActionKinematicConstraint
  
  properties (SetAccess=protected)
    name='';
    body_ind
    body_pts 
    pos_min   % trajectory
    pos_max   % trajectory
    tspan     % 1x2 double
  end
  
  methods
    function obj = ActionKinematicConstraint(body_ind,body_pos,worldpos,tspan,name)
      sizecheck(body_ind,1);
      obj.body_ind = body_ind;
      sizecheck(body_pos,[3 nan]);
      obj.body_pos = body_pos;
      
      mi = size(body_pos,2);
      if isstruct(worldpos)
        if ~isfield(worldpos,'min') || ~isfield(worldpos,'max')
          error('if worldpos is a struct, it must have fields .min and .max');
        end
        minpos=[worldpos.min];  maxpos=[worldpos.max];
      else
        minpos=worldpos; maxpos=worldpos;
      end
      [rows,cols]=size(minpos);
      if (rows ~= 3 && rows ~= 6) error('worldpos must have 3 or 6 rows'); end
      if (body_ind==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
      if (cols~=mi) error('worldpos must have the same number of elements as bodypos'); end
      sizecheck(maxpos,[rows,mi]);
      minpos(isnan(minpos))=-inf;
      maxpos(isnan(maxpos))=inf;
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
  end
  
  methods (Static=true)
    function obj = onGroundConstraint(body_ind,body_pos,tspan,name)
      pos.min = [nan;nan;0];
      pos.max = [nan;nan;0];
      obj = ActionKinematicConstraint(body_ind,body_pos,pos,tspan,name);
    end
  end
end
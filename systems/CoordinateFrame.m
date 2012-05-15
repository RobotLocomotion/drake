classdef CoordinateFrame < handle
% Every input, state, and output in a RobotLibSystem has a coordinate frame
% attached to it.  Many bugs can be avoided by forcing developers to be 
% explicit about these coordinate systems when they make combinations of
% systems.
  
  properties (SetAccess=private,GetAccess=public)
    name;           % string name for this coordinate system
    dim;            % scalar dimension of this coordinate system
    transforms={};  % handles to CoordinateTransform objects

    coordinates={}; % list of coordinate names

    angle_flag=[];  % angle_flag(i)=true iff variable i wraps around 2pi
    poly=[];           % optional msspoly variables for this frame
  end
  
  methods
    function obj=CoordinateFrame(name,dim,prefix)
      typecheck(name,'char');
      obj.name = name;
      
      typecheck(dim,'double');
      sizecheck(dim,[1 1]);
      obj.dim = dim;
      
      if (nargin<3)
        ind = strfind(name,':');
        if isempty(ind)
          prefix = name(1);
        else
          prefix = name(ind(end)+[1:2]);
        end
      else
        typecheck(prefix,'char');
        sizecheck(prefix,[1 1]);
      end
      
      ind=1;
      function str=coordinateName(~);
        str=[prefix,num2str(ind)];
        ind=ind+1;
      end
      obj.coordinates=cellfun(@coordinateName,cell(dim,1),'UniformOutput',false);
      
      if checkDependency('spot_enabled') && dim>0
        if (prefix=='t') error('oops.  destined for a collision with msspoly representing time'); end
        obj.poly = msspoly(prefix,dim);
      end
      
      obj.angle_flag = repmat(false,dim,1);
    end
    
    function obj=addTransform(obj,transform)
      typecheck(transform,'CoordinateTransform');
      if (getInputFrame(transform) ~= obj || getOutputFrame(transform) == obj)
        error('transform must be from this coordinate frame to another to be added');
      end
      if ~isempty(findTransform(obj,getOutputFrame(transform)))
        error('i already have a transform that gets me to that frame');
      end
      obj.transforms{end+1}=transform;
    end
    
    function tf=findTransform(obj,target)
      typecheck(target,'CoordinateFrame');
      ind=find(cellfun(@(a)getOutputFrame(a)==target,obj.transforms));
      if (isempty(ind))
        tf=[];
      else
        tf=obj.transforms{ind};
      end
    end
    
    function obj=setAngleFlags(obj,flags)
      typecheck(flags,'logical');
      sizecheck(flags,[obj.dim,1]);
      obj.angle_flag = flags;
    end
    
    function obj=setCoordinateNames(obj,cnames)
      if (iscell(cnames) && isvector(cnames) && length(cnames)==obj.dim && all(cellfun(@ischar,cnames)))
        obj.coordinates=cnames;
      else
        error('cnames must be a cell vector of length dim populated with strings'); 
      end
    end
  end

  % todo: consider putting LCM encode/decode in here
end

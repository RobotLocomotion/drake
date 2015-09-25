classdef CoordinateFrame < handle
% Every input, state, and output in a DynamicalSystem has a coordinate frame
% attached to it.  Many bugs can be avoided by forcing developers to be
% explicit about these coordinate systems when they make combinations of
% systems.

  properties (SetAccess=private,GetAccess=public)
    name='';        % string name for this coordinate system
    dim=0;          % scalar dimension of this coordinate system
    transforms={};  % handles to CoordinateTransform objects
    prefix;         % a vector character prefix used for the msspoly variables, or a vector of size dim listing prefixes for each variable
  end
  properties (Access=private)
    coordinates={}; % list of coordinate names
    poly=[];        % optional msspoly variables for this frame
  end

  methods
    function obj=CoordinateFrame(name,dim,prefix,coordinates)
      % Constructs a new coordinate frame.
      %
      % @param name is a string name associated with this coordinate frame
      %
      % @param dim is the scalar number of elements in this frame
      %
      % @param prefix is a single character for a mss_poly variable
      % associated with this frame.  you may also specify a vector of size
      % dim of characters specifying the prefix for each variable.
      % @default first character of name
      %
      % @param coordinates is a cell array (with dim elements) of strings
      % of giving the names of the coordinates. @default x1,x2,x3 etc,
      % where x is the prefix
      %
      % @retval obj the newly constructed CoordinateFrame

      typecheck(name,'char');
      obj.name = name;

      typecheck(dim,'double');
      sizecheck(dim,[1 1]);
      obj.dim = dim;

      if (nargin<3 || isempty(prefix))
        ind = strfind(name,':');
        if isempty(ind)
          prefix = name(1);
        else
          prefix = name(ind(end)+[1:2]);
        end
      else
        typecheck(prefix,'char');
        if ~isscalar(prefix)
          sizecheck(prefix,dim);
          prefix = prefix(:);
        end
      end
      if isscalar(prefix)
        prefix = repmat(prefix,dim,1);
      end
      obj.prefix = prefix;

      ind=1;
      function str=coordinateName(~)
        str=[prefix(ind),sprintf('%d',sum(prefix(1:ind)==prefix(ind)))];
        ind=ind+1;
      end
      if (nargin<4 || isempty(coordinates))
        obj.coordinates=cellfun(@coordinateName,cell(dim,1),'UniformOutput',false);
      else
        typecheck(coordinates,'cell');
        sizecheck(coordinates,dim);
        for i=1:dim
          typecheck(coordinates{i},'char');
        end
        obj.coordinates = {coordinates{:}}';
      end
    end
    
    function tf = hasSamePrefix(frame1,frame2)
      % useful for alarming on a possible prefix clash between two polys
      tf = any(any(bsxfun(@eq,frame1.prefix,frame2.prefix')));
    end

    function p = getPoly(obj)
      % create the poly now if it hasn't been created yet
      if obj.dim>0 && isempty(obj.poly)
        checkDependency('spotless');
        if (obj.prefix=='t') error('oops.  destined for a collision with msspoly representing time'); end
        if all(obj.prefix==obj.prefix(1))
          obj.poly=msspoly(obj.prefix(1),obj.dim);
        else
          obj.poly=msspoly('_',obj.dim); % to be replaced below... just initializing
          for u=unique(obj.prefix')
            obj.poly(obj.prefix==u)=msspoly(u,sum(obj.prefix==u));
          end
        end
      end
      p = obj.poly;
    end

    function disp(obj)
      fprintf(1,'Coordinate Frame: %s (%d elements)\n',obj.name,obj.dim);
      for i=1:obj.dim
        fprintf(1,'  %s\n',obj.coordinates{i});
      end
    end

    function tf = isequal_modulo_transforms(a,b)
      % returns true if the two coordinate frames are the same.
      % the "modulo transforms" refers to the fact that two identical
      % frames, A and B, could be different if B knows how to transform to
      % C, but A does not.
      tf = isequal(a.name,b.name) && ...
        isequal(a.dim,b.dim) && ...
        isequal(a.coordinates,b.coordinates) && ...
        isequal(a.prefix,b.prefix);
    end

    function s = getSym(obj)
      for i=1:length(obj.dim)
        s(1) = sym(obj.coordinates{i},'real');
      end
    end

    function addTransform(obj,transform,bforce)
      % Attaches a new coordinate transform from the current frame to a
      % different frame. An error is throw if there already exists any
      % transform (or combination of transforms) that can already transform
      % the current frame to the output frame.
      %
      % @param transform a CoordinateTransform object with the input frame
      % matching this current frame.
      % @param bforce if true, then it forces the addition (overriding the
      % search for existing transforms).  This was added to optimize
      % transform addition for the case when we are sure that no transform
      % already exists.  Use with caution.

      typecheck(transform,'CoordinateTransform');
      if (nargin<3) bforce = false; end
      if (getInputFrame(transform) ~= obj || getOutputFrame(transform) == obj)
        error('Drake:CoordinateFrame:BadTransform','transform must be from this coordinate frame to another to be added');
      end
      if ~bforce && ~isempty(findTransform(obj,getOutputFrame(transform)))
        error('Drake:CoordinateFrame:ExistingTransform','i already have a transform that gets me to that frame');
      end
      obj.transforms{end+1}=transform;
%      drawFrameGraph(obj); keyboard;
    end

    function updateTransform(obj,newtransform)
      % find a current transform with the same target frame and replace it
      % with the new transform.  this only searches simple transforms (from
      % the current frame to the output frame), not multi-hop transforms
      % through multiple frames.

      typecheck(newtransform,'CoordinateTransform');
      ind=find(cellfun(@(a)getOutputFrame(a)==newtransform.getOutputFrame,obj.transforms));
      if (isempty(ind))
        error('no transform to update');
      elseif length(ind)>1
        error('found multiple transforms');
      else
        obj.transforms{ind}=newtransform;
      end
    end

    function addProjectionTransformByCoordinateNames(fr,fr2,fr2_defaultvals)
      % adds a transform from fr to fr2 which copies over the dimensions
      % with matching coordinate names, and sets the remaining elements of
      % fr2 to their default values.
      % @param fr  the coordinate frame which gets the new transform (from
      % this frame)
      % @param fr2 the coordinate frame that the transform maps to.
      % @param fr2_defaultvals a double vector or Point which specifies the
      % constant values of the elements in fr2 which do not get mapped from
      % fr

      typecheck(fr2,'CoordinateFrame');
      if (nargin<3)
        fr2_defaultvals = Point(fr2);
      else
        if ~isa(fr2_defaultvals,'Point')
          fr2_defaultvals = Point(fr2,fr2_defaultvals);
        end
      end
      typecheck(fr2_defaultvals,'Point');
      if (fr2_defaultvals.getFrame()~=fr2)
        error('default values must be in the fr2 frame');
      end

      [lia,locb] = ismember(fr2.coordinates,fr.coordinates);
      T = sparse(find(lia),locb(locb>0),1,fr2.dim,fr.dim);
      b = double(fr2_defaultvals); b(lia)=0;
      tf = AffineTransform(fr,fr2,T,b);

      addTransform(fr,tf);
    end

    function drawFrameGraph(obj)
      % Calls graphviz to visualize this frame plus all other frames that
      % are reachable by some (potentially multi-hop) transform.
      %
      % Note: requires that graphviz dot is installed on the system

      [A,fr] = extractFrameGraph(obj);

      if (size(A,1)<length(A)) A(length(A),end)=0; end
      if (size(A,2)<length(A)) A(end,length(A))=0; end
      drawGraph(A,cellfun(@(a) a.name,fr,'UniformOutput',false));
    end


    function [tf,options]=findTransform(obj,target,options)
      % Performs a simple breadth-first search of all available multi-hop
      % transforms for a transformation from the current frame to the
      % target frame
      %
      % @param target a CoordinateFrame that will be the output of the
      % (possibly compound) CoordinateTransform
      %
      % @option throw_error_if_fail @default false
      % @option depth positive scalar search depth. @default inf
      %
      % @retval tf the resulting (possibly compound) transform, or the
      % empty matrix if no transform was found.
      %
      % @retval the options structure sprinkled with extra information.
      % this output enables recursive calls to the function, and is
      % intended only for internal use.
      %

      if (obj==target)
        % note: it's tempting to replace this with
        % isequal_modulo_transforms, but I think it's more appropriate to
        % keep this more exclusive.  For example, in each call to lqr, I construct
        % multiple frames that are equal by the isequal_modulo_transforms
        % test, but actually point to different coordinates.
        tf = ConstOrPassthroughSystem(repmat(nan,obj.dim,1),obj.dim);
        return;
      end

      if (nargin<3) options=struct(); end
      if ~isfield(options,'throw_error_if_fail') options.throw_error_if_fail = false; end

      typecheck(target,'CoordinateFrame');
      ind=find(cellfun(@(a)getOutputFrame(a)==target,obj.transforms));
      if isempty(ind)
        if ~isfield(options,'depth') options.depth=inf; end
        if ~isfield(options,'dirty_list') options.dirty_list=[]; end
        if ~isfield(options,'queue') options.queue=[]; end
        if ~isfield(options,'tf_from_parent') options.tf_from_parent=[]; end

        tf=[];
        if (options.depth>1)
          options.depth = options.depth-1;

          % add myself to the dirty list
          options.dirty_list = horzcat(options.dirty_list, struct('frame',obj,'tf_from_parent',options.tf_from_parent));

          % add my children to the queue
          options.queue = horzcat(options.queue,struct('frame',cellfun(@getOutputFrame,obj.transforms,'UniformOutput',false),'tf_from_parent',obj.transforms));

          % now check the queue for a match
          while isempty(tf) && ~isempty(options.queue)
            a = options.queue(1);
            options.queue = options.queue(2:end);

            if ismember(a.frame,{options.dirty_list(:).frame})
              continue;
            end

            options.tf_from_parent = a.tf_from_parent;
            [tf,options] = findTransform(a.frame,target,options);
          end

          if ~isempty(tf)  % then reconstruct tf chain
            parent = options.tf_from_parent;
            while ~isempty(parent)
              tf = cascade(parent,tf);
              [~,loc]=ismember(tf.getInputFrame,{options.dirty_list(:).frame});
              parent = options.dirty_list(loc).tf_from_parent;
            end
            return;
          end
        end

        if (nargin>2 && options.throw_error_if_fail)
          error(['Could not find any transform between ',obj.name,' and ', target.name]);
        end
      elseif length(ind)>1
        error(['Found multiple transforms between ',obj.name,' and ', target.name]);
      else
        tf=obj.transforms{ind};
      end
    end

    function str = getCoordinateName(obj,i)
      str = obj.coordinates{i};
    end

    function ind = findCoordinateIndex(obj,varname)
      ind = find(strcmp(varname,obj.coordinates));
    end

    function strs = getCoordinateNames(obj)
      strs = obj.coordinates;
    end

    function setCoordinateNames(obj,cnames)
      % Updates the coordinate names
      %
      % @param cnames must be a cell array vector of length dim populated
      % with strings
      %

      if (iscellstr(cnames) && isvector(cnames) && length(cnames)==obj.dim)
        obj.coordinates=cnames;
      else
        error('cnames must be a cell vector of length dim populated with strings');
      end
    end

    function fr=subFrame(obj,dims)
      % Extracts a new frame with a subset of the original variables, so that
      % fr.coordinates = obj.coordinates(dims)
      %
      % @param dims a numeric vector of index values such that fr.coordinates = obj.coordinates(dims)
      %
      % @retval the newly constructed frame
      %

      if ~isnumeric(dims) || ~isvector(dims) error('dims must be a numeric vector'); end
      if (any(dims>obj.dim | dims<1)) error(['dims must be between 1 and ',obj.dim]); end
      fr = CoordinateFrame([obj.name,mat2str(dims)], length(dims), obj.prefix(dims));
      fr.coordinates = obj.coordinates(dims);
      if ~isempty(fr.poly)
        fr.poly = obj.poly(dims);
      end
    end

    function fr=constructFrameWithAnglesWrapped(obj,angle_flag,q0)
      % produces a copy of the current frame, but with a transform placed
      % between them that wraps the angles around 2pi.  The transform wraps all
      % coordinates with angle_flag = true to be inside [-pi+q0,pi+q0]
      %
      % @param angle_flags boolean vector of length obj.dim which is true
      % for each coordinate that will be wrapped around 2*pi.
      %
      % @param q0 double vector of with the default angle around which to
      % perform the wrapping.  it should be the length of the number of
      % wrapped angles (e.g., so that x(angle_flags) = q0).

      fr = CoordinateFrame([obj.name,'Wrapped'],obj.dim,obj.prefix,obj.coordinates);

      if (nargin>2)
        obj.addTransform(AngleWrappingTransform(obj,fr,angle_flag,q0));
      else
        obj.addTransform(AngleWrappingTransform(obj,fr,angle_flag));
      end
      fr.addTransform(AffineTransform(fr,obj,eye(obj.dim),zeros(obj.dim,1)));
    end

    function scope(obj,t,val,options)
      % publishes coordinate information to the lcm scope
      if (nargin<4) options=struct(); end
      for i=1:length(obj.dim)
        scope(obj.name,obj.coordinates{i},t,val(i),options);
      end
    end

    function generateLCMType(obj,robot_name,signal_name)
      % writes an lcm type specification to file from the coordinate frame
      % description. not to be used frequently.

      if (nargin<3)
        name = lower(obj.name);
      else
        name = [lower(robot_name),'_',lower(signal_name)];
      end
      typename = ['lcmt_',name];
      fname = [typename,'.lcm'];
      if strcmpi(input(['About to write file ',fname,' .  Should I proceed (y/n)? '],'s'),'y')
        fptr=fopen(fname,'w');

        fprintf(fptr,'// Note: this file was automatically generated using the\n// CoordinateFrame.generateLCMType() method.\n\n');
        fprintf(fptr,'struct %s\n{\n  int64_t timestamp;\n\n',typename);
        for i=1:obj.dim
          fprintf(fptr,'  double %s;\n',CoordinateFrame.stripSpecialChars(obj.coordinates{i}));
        end
        fprintf(fptr,'}\n\n');
        fclose(fptr);
        disp('file written successfully.  done.');
      end
    end
  end

  methods  % some functions which help operations MultiCoordinateFrames
    function n = getNumFrames(obj)
      n = 1;
    end

    function fr = getFrameByNum(obj,n)
      if (n==1) fr = obj; else error('bad frame num'); end
    end

    function id = getFrameNum(obj,fr)
      if (fr==obj) id=1;
      else error('Drake:CoordinateFrame:NoFrame', 'can''t find frame %s',fr.name); end
    end

    function insys=setupMultiInput(obj,mdl,subsys)
      insys=subsys;
    end

    function outsys=setupMultiOutput(obj,mdl,subsys)
      outsys=subsys;
    end

    function setupLCMInputs(obj,mdl,subsys,subsys_portnum,options)
      typecheck(mdl,{'char','SimulinkModelHandle'});
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double');
      add_block('simulink3/Sources/In1',[mdl,'/in',uid]);
      add_line(mdl,['in',uid,'/1'],[subsys,'/',num2str(subsys_portnum)]);
    end

    function setupLCMOutputs(obj,mdl,subsys,subsys_portnum,options)
      typecheck(mdl,{'char','SimulinkModelHandle'});
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double');
      add_block('simulink3/Sources/Terminator',[mdl,'/terminator',uid]);
      add_line(mdl,[subsys,'/',num2str(subsys_portnum)],['terminator',uid,'/1']);
    end

    function connection = autoConnect(fr1,fr2,connection)
      % populates the connection structure as used in mimoCascade and
      % mimoFeedback
      % if connection is passed in, then it simply attempts to validate the
      % connection.

      if nargin>2 && ~isempty(connection)
        typecheck(connection,'struct');
        if ~isempty(setxor(fieldnames(connection),{'from_output','to_input'}))
          error('connection must be a struct with fields "from_output" and "to_input"');
        end
        % convert any frames to indices
        for i=1:length(connection)
          if isa(connection(i).from_output,'CoordinateFrame')
            connection(i).from_output = getFrameNum(fr1,connection(i).from_output);
          end
          if isa(connection(i).to_input,'CoordinateFrame')
            connection(i).to_input = getFrameNum(fr2,connection(i).to_input);
          end
          typecheck(connection(i).from_output,'numeric');
          typecheck(connection(i).to_input,'numeric');
        end
        rangecheck([connection.from_output],1,getNumFrames(fr1));
        rangecheck([connection.to_input],1,getNumFrames(fr2));
      else
        connection=[];
        for i=1:getNumFrames(fr1)
          for j=1:getNumFrames(fr2)
            if getFrameByNum(fr1,i)==getFrameByNum(fr2,j)
              tf=true;  % not actually used here, just make it non-empty
            else
              tf=findTransform(getFrameByNum(fr1,i),getFrameByNum(fr2,j));
            end
            if ~isempty(tf)
              if ~isempty(connection)&&any([connection.to_input]==j)
                error('Automatic connection failed.  The possible mappings from the output of sys1 to the input of sys2 are not unique');
              end
              connection(end+1).from_output=i;
              connection(end).to_input=j;
            end
          end
        end
        if isempty(connection)
          error('Automatic connection failed.  Could not find any possible connections between sys1 and sys2');
        end
      end
    end

  end

  methods (Access=protected)
    function [tf,loc]=ismember(obj,cell_of_frames)
      % helper method for searching transforms
      typecheck(cell_of_frames,'cell');
      loc=find(cellfun(@(a) a==obj,cell_of_frames));
      tf = ~isempty(loc);
    end
  end

  methods (Access=protected)
    function [A,fr] = extractFrameGraph(obj)
      A = 0;
      fr = {obj};
      [A,fr]=recurseTFs(obj,A,fr);

      function [A,fr]=recurseTFs(obj,A,fr)
        [~,ind]=ismember(obj,fr);
        children = cellfun(@getOutputFrame,obj.transforms,'UniformOutput',false);
        for i=1:length(children)
          [b,cind]=ismember(children{i},fr);
          if b
            A(ind,cind)=1;
            continue;
          else
            fr=vertcat(fr,children(i));
            A(ind,end+1)=1;
            [A,fr]=recurseTFs(children{i},A,fr);
          end
        end
      end
    end
  end

  methods (Static=true,Hidden=true)
    function s=stripSpecialChars(s)
      s=regexprep(s,'\\','');
    end
  end
end

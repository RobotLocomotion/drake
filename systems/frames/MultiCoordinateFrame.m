classdef MultiCoordinateFrame < CoordinateFrame
  % Makes a mux'd coordinate frame out of a list of other frames.  This is
  % the means by which drake supports (the simplest form of) multi-input
  % and multi-output systems.
  %
  % Note that, although the coordinate names (and polynomial variables) are
  % initialized to match the name of the sub coordinate frames, there is 
  % no contract guaranteeing that they will continue to match after 
  % subsequent coordinate renamings for the multi-frame or the sub-frames.
  
  properties
    frame={};     % a list of CoordinateFrame objects
    frame_id=[];  % for each coordinate, an integer index into the frame 
                  % (from the list above) associated with that coordinate
    coord_ids={}; % for each frame, a list of associate coordinates   
    
    % speed optimization
    cell_vals={};
  end

  methods
    function obj = MultiCoordinateFrame(coordinate_frames,frame_id)
      if (nargin<2) frame_id = []; end
      
      typecheck(coordinate_frames,'cell');
      name=[];
      dim=0;
      prefix=[];
      coordinates={};
      
      if (length(coordinate_frames)==1)
        error('you''ve only passed in a single frame.  you should be calling MultiCoordinateFrame.constructFrame instead, to get the correct behavior');
      end
      
      if isempty(frame_id)
        for i=1:length(coordinate_frames)
          frame_id = vertcat(frame_id,repmat(i,coordinate_frames{i}.dim,1));
        end
      else
        rangecheck(frame_id,1,length(coordinate_frames));
      end
      
      % if coordinate_frame contains multi-frames, then extract them here
      % (don't allow recursive multi-frames)
      cf = coordinate_frames;
      coordinate_frames={};
      for i=length(cf):-1:1
        if isa(cf{i},'MultiCoordinateFrame')
          coordinate_frames=vertcat(cf{i}.frame,coordinate_frames);
          if ~isempty(frame_id)
            % update frame ids accordingly (by wedging these frames in)
            frame_id(frame_id>i) = frame_id(frame_id>i)+length(cf{i}.frame)-1;
            frame_id(frame_id==i) = cf{i}.frame_id + i-1;
          end
        else
          coordinate_frames=vertcat({cf{i}},coordinate_frames);
        end
      end
      
      for i=1:length(coordinate_frames)
        typecheck(coordinate_frames{i},'CoordinateFrame');
        name = [name,'+',coordinate_frames{i}.name];
        dim = dim+coordinate_frames{i}.dim;
        prefix = vertcat(prefix,coordinate_frames{i}.prefix);
        coordinates(frame_id==i) = reshape(coordinate_frames{i}.coordinates,[],1);
        coord_ids{i} = find(frame_id==i);
      end
      name=name(2:end); % remove extra '+'
      
      sizecheck(frame_id,dim);
      obj = obj@CoordinateFrame(name,dim,prefix,coordinates);
      obj.frame = coordinate_frames;
      obj.frame_id = frame_id;
      obj.coord_ids = coord_ids;
      
      % add a transform from this multiframe to the child frame
      % iff the subframes are unique
      for i=1:length(coordinate_frames)
        d = coordinate_frames{i}.dim;
        
        if ~any(cellfun(@(a) a==coordinate_frames{i},coordinate_frames([1:i-1,i+1:end])))
          T = sparse(1:d,obj.coord_ids{i},1,d,dim);
          tf = AffineTransform(obj,coordinate_frames{i},T,zeros(d,1));
          addTransform(obj,tf);
        end
      end
      
      % preallocate
      obj.cell_vals = cell(1,length(obj.frame));
      for i=1:length(obj.frame)
        obj.cell_vals{i} = zeros(numel(obj.coord_ids{i}),1);
      end
    end
    
    function disp(obj)
      fprintf(1,'Multi-Coordinate Frame: %s (%d elements)\n',obj.name,obj.dim);
      for i=1:length(obj.frame)
        disp(obj.frame{i});
      end
    end
    
    function display(obj)
      fprintf(1,'Multi-Coordinate Frame: %s (%d elements)\n',obj.name,obj.dim);
      for i=1:length(obj.frame)
        fprintf(1,'  %s (%d elements)\n',obj.frame{i}.name, obj.frame{i}.dim);
      end
    end
    
    function tf = isequal_modulo_transforms(a,b)
      tf = isequal_modulo_transforms@CoordinateFrame(a,b) && ...
        isequal(a.frame_id,b.frame_id) && ...
        isequal(a.coord_ids,b.coord_ids) && ...
        isequal(size(a.frame),size(b.frame));
      for i=1:length(a.frame)
        tf = tf && isequal_modulo_transforms(a.frame{i},b.frame{i});
      end
    end
    
    function tf = findTransform(obj,target,options)
      % There are two ways to get a transform from this multiframe to
      % another frame.  One is if a transform exists directly from the
      % multi-frame.  The other is if the required transforms exist for ALL
      % of the child frames.  see the mimoCascade and mimoFeedback methods
      % for more complex combinations.

      if (nargin<3) options=struct(); end
      if ~isfield(options,'throw_error_if_fail') options.throw_error_if_fail = false; end

      opt2 = options;
      opt2.throw_error_if_fail=false;
      tf = findTransform@CoordinateFrame(obj,target,opt2);
      if isempty(tf) && isa(target,'MultiCoordinateFrame')
        % this could only happen if the target is also a multiframe
        
        % handle the simple case first, where the number of subframes is 
        % the same and obj frame i has a transform to target frame i.
        tf=[];
        if getNumFrames(obj)==getNumFrames(target)
          for i=1:length(obj.frame)
            tfi = findTransform(obj.frame{i},getFrameByNum(target,i),opt2);
            if isempty(tfi)
              tf=[];
              fr2=getFrameByNum(target,i);
              warning(['Could not find any transform between ',obj.frame{i}.name,' and ', fr2.name]);
              break;
            elseif isempty(tf)
              tf = tfi;
            else
              tf = parallel(tf,tfi);
            end
          end
        end
      end
      
      if isempty(tf) && options.throw_error_if_fail
        error(['Could not find any transform between ',obj.name,' and ', target.name]);
      end
    end
    
    function [tf,fid] = findChildTransform(obj,target,options)
      % note: if target is one of the children, then this will return an
      % empty tf but an fid>0
      if (nargin<3) options=struct(); end
      childoptions = options;
      childoptions.throw_error_if_fail = false;
      tf=[];
      fid=-1;
      for i=1:length(obj.frame)
        if (obj.frame{i}==target)
          if (fid>0)
            error('Found transforms to this target from multiple children.  That''s not allowed');
          end
          fid = i;
        else
          thistf = findTransform(obj.frame{i},target,childoptions);
          if ~isempty(thistf)
            if fid<0
              tf = thistf;
              fid = i;
            else
              error('Found transforms to this target from multiple children.  That''s not allowed');
            end
          end
        end
      end
      if (fid<1 && options.throw_error_if_fail)
        error('Couldn''t find a transform from any of my children to this target');
      end       
    end

    function [tf,fid] = findChildReverseTransform(obj,origin,options)
      % finds a transform from origin to one of my children
      % note: if origin is one of the children, then this will return an
      % empty tf but an fid>0

      if (nargin<3) options=struct(); end
      childoptions = options;
      childoptions.throw_error_if_fail = false;
      tf=[];
      fid=-1;
      for i=1:length(obj.frame)
        if (obj.frame{i}==origin)
          if (fid>0)
            error('Found transforms from this origin to multiple children.  That''s not allowed');
          end
          fid = i;
        else
          thistf = findTransform(origin,obj.frame{i},childoptions);
          if ~isempty(thistf)
            if fid<0
              tf = thistf;
              fid = i;
            else
              error('Found transforms from this origin to multiple children.  That''s not allowed');
            end
          end
        end
      end
      if (fid<1 && options.throw_error_if_fail)
        error('Couldn''t find a transform from this origin to any of my children');
      end       
    end
    
    function fr = subFrameByFrameID(obj,frame_ids)
      % comparable to subFrame, but at the frame ID 
      
      if ~isnumeric(frame_ids) || ~isvector(frame_ids) error('frame_ids must be a numeric vector'); end
      if (any(frame_ids>length(obj.frame) | frame_ids<1)) error(['frame_ids must be between 1 and ',length(obj.frame)]); end
      if (length(frame_ids)==1)
        fr = obj.frame{frame_ids};
      else
        fr = MultiCoordinateFrame({obj.frame{frame_ids}});
      end
    end
    
    function varargout = splitCoordinates(obj,vector_vals)
      % Extract values of individual CoordinateFrames from a vector in the
      % multi-frame.  
      % @param vector_vals a vector in the multi-frame
      %  
      % Usage:  cv = splitCoordinates(obj,vv);
      %   a = cv{1}; b=cv{2};
      % or 
      %   [a,b] = splitCoordinates(obj,vv)
      
      for i=1:length(obj.frame)
        obj.cell_vals{i} = vector_vals(obj.coord_ids{i});
      end
      if nargout==1
        varargout = {obj.cell_vals};
      else
        varargout = obj.cell_vals;
      end
    end      
    
    function vector_vals = mergeCoordinates(obj,cell_vals)
%      sizecheck(cell_vals,length(obj.frame));  % commented out for speed
      vector_vals = cell_vals{1}(1)*zeros(obj.dim,1);
      for i=1:length(obj.frame)
        vector_vals(obj.coord_ids{i}) = cell_vals{i};
      end
    end
    
    function n = getNumFrames(obj)
      n = length(obj.frame);
    end

    function fr = getFrameByNum(obj,n)
      rangecheck(n,1,length(obj.frame));
      fr = obj.frame{n};
    end

    function fr = getFrameByName(obj,name)
      id = find(cellfun(@(a)strcmp(a.name,name),obj.frame));
      if length(id)~=1
        error(['couldn''t find unique frame named ',name]);
      end
      fr = obj.frame{id};
    end
    
    function id = getFrameNum(obj,frame)
      id = find(cellfun(@(a)isequal(a,frame),obj.frame));
      if length(id)>1
        error('frame matched multiple children.  child frames must be unique, otherwise the behavior could get confusing fast');
      end
    end

    function str = getCoordinateName(obj,i)
      ind = obj.frame_id(i);
      str = getCoordinateName(obj.frame{ind},find(obj.coord_ids{ind}==i));
    end
    
    function insys=setupMultiInput(obj,mdl,subsys)
      if ~valuecheck(obj.frame_id,sort(obj.frame_id))  % assume that the simple ordering is ok
        insys = [subsys,'inselector'];
        tmp = [obj.coord_ids{:}]; [~,ind] = sort(tmp);
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',insys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str(ind));
        add_line(mdl,[insys,'/1'],[subsys,'/1']);
        subsys = insys;
      end
      insys = [subsys,'mux'];
      add_block('simulink3/Signals & Systems/Mux',[mdl,'/',insys],'Inputs',mat2str(cellfun(@(a) a.dim,obj.frame)));
      add_line(mdl,[insys,'/1'],[subsys,'/1']);
    end
    
    function outsys=setupMultiOutput(obj,mdl,subsys)
      if ~valuecheck(obj.frame_id,sort(obj.frame_id))
        outsys = [subsys,'outselector'];
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',outsys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str([obj.coord_ids{:}]));
        add_line(mdl,[subsys,'/1'],[outsys,'/1']);
        subsys = outsys;
      end      
      outsys = [subsys,'demux'];
      add_block('simulink3/Signals & Systems/Demux',[mdl,'/',outsys],'Outputs',mat2str(cellfun(@(a) a.dim,obj.frame)));
      add_line(mdl,[subsys,'/1'],[outsys,'/1']);
    end
    
    function setupLCMInputs(obj,mdl,subsys,subsys_portnum,options)
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      if ~valuecheck(obj.frame_id,sort(obj.frame_id))
        insys = [subsys,'inselector'];
        tmp = [obj.coord_ids{:}]; [~,ind] = sort(tmp);
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',insys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str(ind));
        add_line(mdl,[insys,'/1'],[subsys,'/',num2str(subsys_portnum)]);
        subsys = insys; subsys_portnum=1;
      end      
      add_block('simulink3/Signals & Systems/Mux',[mdl,'/mux',uid],'Inputs',num2str(length(obj.frame)));
      for i=1:length(obj.frame)
        setupLCMInputs(obj.frame{i},mdl,['mux',uid],i,options);
      end
      add_line(mdl,['mux',uid,'/1'],[subsys,'/',num2str(subsys_portnum)]);
    end
    
    function setupLCMOutputs(obj,mdl,subsys,subsys_portnum,options)
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      if ~valuecheck(obj.frame_id,sort(obj.frame_id)) 
        outsys = [subsys,'outselector'];
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',outsys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str([obj.coord_ids{:}]));
        add_line(mdl,[subsys,'/',subsys_portnum],[outsys,'/1']);
        subsys = outsys; subsys_portnum=1;
      end      
      add_block('simulink3/Signals & Systems/Demux',[mdl,'/demux',uid],'Outputs',mat2str([obj.frame{:}.dim]));
      for i=1:length(obj.frame)
        setupLCMOutputs(obj.frame{i},mdl,['demux',uid],i,options);
      end
      add_line(mdl,[subsys,'/',num2str(subsys_portnum)],['demux',uid,'/1']);
    end    
    
  end
  
  methods (Access=protected)
    function [A,fr] = extractFrameGraph(obj)
      [A,fr] = extractFrameGraph@CoordinateFrame(obj);
      for i=1:length(obj.frame)
        if ~ismember(obj.frame{i},fr)
          [B,frb] = extractFrameGraph(obj.frame{i});
          A = blkdiag(A,B);
          fr = vertcat(fr,frb);
        end
      end
    end
  end
  
  methods (Static=true)
    function obj = constructFrame(frames,frame_ids,zap_empty_frames)
      % if frames has only a single element, then return it, otherwise
      % construct the construct the mimo frame
      typecheck(frames,'cell');
      
      if (nargin<2) frame_ids =[]; end
      if (nargin<3) zap_empty_frames = false; end
      
      if (nargin>1 && zap_empty_frames)
        i=1;
        while (i<=length(frames))  % zap empty frames
          if (frames{i}.dim<1)
            if any(frame_ids==i) error('bad frame_ids'); end
            frames=frames([1:i-1,i+1:end]);
            frame_ids(frame_ids>i)=frame_ids(frame_ids>i)-1;  % note: written to be ok if frame_ids is empty
          else
            i=i+1;
          end
        end
      end
      
      if length(frames)<1
        obj = CoordinateFrame('EmptyFrame',0);
      elseif (length(frames)==1)
        obj = frames{1};
      else
        obj = MultiCoordinateFrame(frames,frame_ids);
      end
    end
  end
end
  

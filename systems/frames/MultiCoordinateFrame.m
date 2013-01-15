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
  end

  methods
    function obj = MultiCoordinateFrame(coordinate_frames)
      typecheck(coordinate_frames,'cell');
      name=[];
      dim=0;
      prefix=[];
      coordinates={};
      for i=1:length(coordinate_frames)
        typecheck(coordinate_frames{i},'CoordinateFrame');
        name = [name,'+',coordinate_frames{i}.name];
        dim = dim+coordinate_frames{i}.dim;
        prefix = vertcat(prefix,coordinate_frames{i}.prefix);
        coordinates = vertcat(coordinates,coordinate_frames{i}.coordinates);
      end
      name=name(2:end);
      
      obj = obj@CoordinateFrame(name,dim,prefix,coordinates);
      obj.frame = coordinate_frames;
      obj.frame_id = [];
      for i=1:length(coordinate_frames)
        d = coordinate_frames{i}.dim;
        obj.coord_ids{i} = (1:d) + length(obj.frame_id);
        obj.frame_id = vertcat(obj.frame_id,repmat(i,d,1));

        % add a transform from this multiframe to the child frame
        T = sparse(1:d,obj.coord_ids{i},1,d,dim);
        tf = AffineTransform(obj,coordinate_frames{i},T,zeros(d,1));
        addTransform(obj,tf);
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
    
    function [fr,indices] = subFrameByFrameID(obj,frame_ids)
      % comparable to subFrame, but at the frame ID 
      
      if ~isnumeric(frame_ids) || ~isvector(frame_ids) error('frame_ids must be a numeric vector'); end
      if (any(frame_ids>length(obj.frame) | frame_ids<1)) error(['frame_ids must be between 1 and ',length(obj.frame)]); end
      if (length(frame_ids)==1)
        fr = obj.frame{frame_ids};
      else
        fr = MultiCoordinateFrame({obj.frame{frame_ids}});
      end
    end
    
    function cell_vals = splitCoordinates(obj,vector_vals)
      cell_vals = cell(1,length(obj.frame));
      for i=1:length(obj.frame)
        cell_vals{i} = vector_vals(obj.coord_ids{i});
      end
    end      
    
    function vector_vals = mergeCoordinates(obj,cell_vals)
      sizecheck(cell_vals,length(obj.frame));
      vector_vals = cell_vals{1}(1)*zeros(obj.dim,1);
      for i=1:length(obj.frame)
        vector_vals(obj.coord_ids{i}) = cell_vals{i};
      end
    end
    
    function setupLCMInputs(obj,mdl,subsys,subsys_portnum)
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      valuecheck(obj.frame_id,sort(obj.frame_id));  % assume that the simple ordering is ok
      add_block('simulink3/Signals & Systems/Mux',[mdl,'/mux',uid],'Inputs',num2str(length(obj.frame)));
      for i=1:length(obj.frame)
        setupLCMInputs(obj.frame{i},mdl,['mux',uid],i);
      end
      add_line(mdl,['mux',uid,'/1'],[subsys,'/',num2str(subsys_portnum)]);
    end
    
    function setupLCMOutputs(obj,mdl,subsys,subsys_portnum)
      typecheck(mdl,'char');
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      valuecheck(obj.frame_id,sort(obj.frame_id));  % assume that the simple ordering is ok
      add_block('simulink3/Signals & Systems/Demux',[mdl,'/demux',uid],'Outputs',num2str(length(obj.frame)));
      for i=1:length(obj.frame)
        setupLCMOutputs(obj.frame{i},mdl,['demux',uid],i);
      end
      add_line(mdl,[subsys,'/',num2str(subsys_portnum)],['demux',uid,'/1']);
    end    
    
  end
  
  methods (Access=protected)
    function [A,fr] = extractFrameGraph(obj)
      [A,fr] = extractFrameGraph@CoordinateFrame(obj);
      for i=1:length(obj.frame)
        [B,frb] = extractFrameGraph(obj.frame{i});
        A = blkdiag(A,B);
        fr = vertcat(fr,frb);
      end
    end
  end
end
  
classdef CoordinateFrameTest < DrakeMexPointer
% Every input, state, and output in a DynamicalSystem has a coordinate frame
% attached to it.  Many bugs can be avoided by forcing developers to be
% explicit about these coordinate systems when they make combinations of
% systems.

%  properties (Access=private)
%    transforms={};  % handles to CoordinateTransform objects
%    poly=[];        % optional msspoly variables for this frame
%  end

  methods
    function obj=CoordinateFrameTest(name,dim,prefix,coordinates)
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
      typecheck(dim,'double');
      sizecheck(dim,[1 1]);

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

      ind=1;
      function str=coordinateName(~)
        str=[prefix(ind),sprintf('%d',sum(prefix(1:ind)==prefix(ind)))];
        ind=ind+1;
      end
      if (nargin<4 || isempty(coordinates))
        coordinates=cellfun(@coordinateName,cell(dim,1),'UniformOutput',false);
      else
        typecheck(coordinates,'cell');
        sizecheck(coordinates,dim);
        for i=1:dim
          typecheck(coordinates{i},'char');
        end
        coordinates = {coordinates{:}}';
      end
      
      mex_ptr_args = cell(1,3);
      [mex_ptr_args{:}] = CoordinateFrameTest.new(name,dim,prefix(1),coordinates);
      obj = obj@DrakeMexPointer(mex_ptr_args{:});
    end
  end
  
  methods (Static)
    varargout = new(varargin)
  end
end
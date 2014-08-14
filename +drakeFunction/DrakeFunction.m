classdef DrakeFunction
  % drakeFunction.DrakeFunction   Abstract parent class for all DrakeFunction classes
  % A DrakeFunction represents a vector-valued function that maps a
  % point in one CoordinateFrame to a point in another CoordinateFrame
  
  properties (SetAccess = immutable)
    input_frame   % CoordinateFrame representing the domain
    output_frame  % CoordinateFrame representing the range
  end

  methods (Abstract)
    % All child classes must implement an 'eval' method that evaluates
    % the function and its derivative
    [f,df] = eval(obj,x);
  end

  methods
    function obj = DrakeFunction(input_frame,output_frame)
      % obj = drakeFunction.DrakeFunction(input_frame,output_frame)
      %   returns a DrakeFunction object representing a map from
      %   input_frame to output_frame
      %
      % @param input_frame    -- CoordinateFrame representing the domain
      % @param output_frame   -- CoordinateFrame representing the range
      %
      % @retval obj           -- drakeFunction.DrakeFunction object

      typecheck(input_frame,'CoordinateFrame');
      typecheck(output_frame,'CoordinateFrame');
      obj.input_frame = input_frame;
      obj.output_frame = output_frame;
    end

    function fcn = plus(obj,other,same_input)
      import drakeFunction.*
      if nargin < 3, same_input = false; end
      if isa(other,'drakeFunction.DrakeFunction')
        fcn = compose(Sum(obj.output_frame,2),Concatenated({obj,other},same_input));
      else
        error('Drake:drakeFunction:NotSupported', ...
          'Addition of DrakeFunctions with other classes is not supported');
      end
    end

    function fcn = minus(obj,other,varargin)
      fcn = plus(obj,-other,varargin{:});
    end

    function fcn = uminus(obj)
      import drakeFunction.*
      fcn = compose(ConstantMultiple(obj.getOutputFrame, ...
                                      obj.getOutputFrame,-1),obj);
    end

    function fcn = vertcat(varargin)
      fcn = drakeFunction.Concatenated(varargin);
    end

    function fcn3 = compose(fcn1, fcn2)
      fcn3 = drakeFunction.Composed(fcn1,fcn2);
    end

    function fcn = duplicate(obj,n)
      % fcn = duplicate(obj,n) returns a concatenated DrakeFunction containing
      % n duplicates of obj
      fcn = drakeFunction.Concatenated(repmat({obj},1,n));
    end

    function fcn = addInputFrame(obj,frame,append)
      % fcn = appendInputFrame(obj,frame,append) returns a DrakeFunction that
      % is the same as obj, but with unused inputs added. These inputs are
      % appended or prepended based on the value of 'append'.
      %
      % @param obj
      % @param frame
      % @param append -- Logical indicating whether to add the inputs after (T)
      %                  or before (F) the current inputs. 
      %                  Optional. @default True 
      if nargin < 3, append = true; end
      dummy_fcn = drakeFunction.Zeros(frame,drakeFunction.frames.R(0));
      if append
        fcn = [obj;dummy_fcn];
      else
        fcn = [dummy_fcn;obj];
      end
    end

    function varargout = subsref(obj,s)
      varargout = cell(1,max(nargout,1));
      switch s(1).type
        case '()'
          if numel(s) < 2
            if isa(s.subs{1},'drakeFunction.DrakeFunction')
              [varargout{:}] = compose(obj,s.subs{1});
            else
              if isa(s.subs{1},'Point')
                assert(isequal_modulo_transforms(s.subs{1}.frame,obj.input_frame));
                x = double(s.subs{1});
              elseif isnumeric(s.subs{1})
                sizecheck(s.subs{1},[obj.input_frame.dim,1]);
                x = s.subs{1};
              else
                error('Drake:drakeFunction:DrakeFunction:UnsupportedType', ...
                  'fcn(x) does not support x of class %s',class(s.subs{1}));
              end
              [varargout{:}] = obj.eval(x);
            end
          else
            error('Drake:drakeFunction:DrakeFunction:TooManySubscripts', ...
                  'fcn(x) only supports one argument');
          end
        otherwise
          [varargout{:}] = builtin('subsref',obj,s);
      end
    end

    function input_frame = getInputFrame(obj)
      input_frame = obj.input_frame;
    end

    function output_frame = getOutputFrame(obj)
      output_frame = obj.output_frame;
    end
  end
end

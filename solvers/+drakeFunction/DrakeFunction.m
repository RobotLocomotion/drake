classdef DrakeFunction
  % drakeFunction.DrakeFunction   Abstract parent class for all DrakeFunction classes
  % A DrakeFunction represents a vector-valued function that maps a
  % point in one CoordinateFrame to a point in another CoordinateFrame
  
  properties (SetAccess = protected)
    input_frame   % CoordinateFrame representing the domain
    output_frame  % CoordinateFrame representing the range

    % gradient sparsity information
    iCfun   % An int vector. The row index of non-zero entries of the gradient matrix
    jCvar   % An int vector. The column index of the non-zero entries of the gradient matrix
    nnz     % An int scalar. The maximal number of non-zero entries in the gradient matrix
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
      obj = obj.setSparsityPattern();
    end

    function [iCfun, jCvar] = getSparsityPattern(obj)
      iCfun = obj.iCfun;
      jCvar = obj.jCvar;
    end

    function obj = setSparsityPattern(obj)
      % obj = setSparsityPattern(obj) sets the sparsity pattern properties
      %   (iCfun and jGvar) in the returned object. Sub-classes that wish to
      %   modify the default sparsity pattern (dense) should overload this
      %   method
      n_input = obj.getInputFrame().dim;
      n_output = obj.getOutputFrame().dim;
      obj.iCfun = reshape(bsxfun(@times,(1:n_output)',ones(1,n_input)),[],1);
      obj.jCvar = reshape(bsxfun(@times,1:n_input,ones(n_output,1)),[],1);
    end

    function fcn = plus(obj,other,same_input)
      % fcn = plus(obj,other,same_input) returns a DrakeFunction whose output
      %   is the sum of the outputs of obj and other. If same_input = true, the
      %   input frame of the returned DrakeFunction is the same as the input
      %   frames of obj and other. If same_input = false, the input frame of
      %   the returned DrakeFunction is the concatenation of their input
      %   frames.
      %
      % fcn = plus(obj,other,same_input) is equivalent to plus(obj,other,false)
      %
      % @param obj          -- DrakeFunction object
      % @param other        -- DrakeFunction object
      % @param same_input   -- Logical scalar indicating whether the functions
      %                        to be summed should share the same input.
      %                        Optional. @default false
      %
      % @retval fcn         -- DrakeFunction which evaluates the sum of obj and
      %                        other
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
      % fcn = minus(obj,other,same_input) See documentation for 
      %   drakeFunction.DrakeFunction.plus
      %
      % @param obj          -- DrakeFunction object
      % @param other        -- DrakeFunction object
      % @param same_input   -- Logical scalar indicating whether the functions
      %
      % @retval fcn         -- DrakeFunction which evaluates the 
      %                        difference between obj and
      %                        other
      fcn = plus(obj,-other,varargin{:});
    end

    function fcn = uminus(obj)
      import drakeFunction.*
      fcn = compose(ConstantMultiple(obj.getOutputFrame, ...
                                      obj.getOutputFrame,-1),obj);
    end

    function fcn = mtimes(obj,other)
      if isnumeric(obj) && isscalar(obj)
        fcn = obj.*other;
      elseif isnumeric(other) && isscalar(other)
        fcn = obj.*other;
      else
        error('Drake:drakeFunction:DrakeFunction:NoMatrixProducts', ...
          'Matrix products of expressions are not yet imiplemented');
      end
    end

    function fcn = times(obj,other)
      import drakeFunction.ConstantMultiple
      if isnumeric(obj)
        fcn_orig = other;
        value = obj;
      elseif isnumeric(other)
        fcn_orig = obj;
        value = other;
      else
        error('Drake:drakeFunction:DrakeFunction:NoDrakeFunctionProducts', ...
          'Elementwise products of two DrakeFunctions are not yet imiplemented');
      end
      frame = fcn_orig.getOutputFrame();
      fcn = compose(ConstantMultiple(frame,value),fcn_orig);
    end

    function fcn = vertcat(obj,varargin)
      fcn = concatenate(obj,varargin{:});
    end

    function fcn = concatenate(obj,varargin)
      if islogical(varargin{end})
        same_input = varargin{end};
        fcns = [{obj}, varargin(1:end-1)];
      else
        same_input = false;
        fcns = [{obj}, varargin];
      end
      fcn = drakeFunction.Concatenated(fcns,same_input);
    end

    function fcn3 = compose(fcn1, fcn2)
      fcn3 = drakeFunction.Composed(fcn1,fcn2);
    end

    function fcn = duplicate(obj,n)
      % fcn = duplicate(obj,n) returns a concatenated DrakeFunction containing
      % n duplicates of obj
      more_obj = repmat({obj},1,n-1);
      fcn = concatenate(obj,more_obj{:});
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
      dummy_fcn = drakeFunction.Zeros(frame,drakeFunction.frames.realCoordinateSpace(0));
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
              [varargout{:}] = eval(obj,x);
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

    function n_inputs = getNumInputs(obj)
      n_inputs = obj.input_frame.dim;
    end

    function n_outputs = getNumOutputs(obj)
      n_outputs = obj.output_frame.dim;
    end
  end
end

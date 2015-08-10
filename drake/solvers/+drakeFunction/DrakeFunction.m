classdef DrakeFunction
  % drakeFunction.DrakeFunction   Abstract parent class for all DrakeFunction classes
  % A DrakeFunction represents a vector-valued function of a single vector input
  
  properties (SetAccess = protected)
    dim_input   % length of input vector
    dim_output   % length of output vector

    % gradient sparsity information
    iCfun   % An int vector. The row index of non-zero entries of the gradient matrix
    jCvar   % An int vector. The column index of the non-zero entries of the gradient matrix
    nnz     % An int scalar. The maximal number of non-zero entries in the gradient matrix
  end

  properties
    name = '';
  end

  methods (Abstract)
    % All child classes must implement an 'eval' method that evaluates
    % the function and its derivative
    [f,df] = eval(obj,x);
  end

  methods
    function obj = DrakeFunction(dim_input, dim_output)
      % obj = drakeFunction.DrakeFunction(dim_input, dim_output)
      %   returns a DrakeFunction object representing a map from
      %   R^dim_input to R^dim_output
      %
      % @param dim_input   % length of input vector
      % @param dim_output   % length of output vector
      %
      % @retval obj           -- drakeFunction.DrakeFunction object

      obj.dim_input = dim_input;
      obj.dim_output = dim_output;
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
      obj.iCfun = reshape(bsxfun(@times,(1:obj.dim_output)',ones(1,obj.dim_input)),[],1);
      obj.jCvar = reshape(bsxfun(@times,1:obj.dim_input,ones(obj.dim_output,1)),[],1);
    end

    function fcn = plus(obj,other,same_input)
      % fcn = plus(obj,other,same_input) returns a DrakeFunction whose output
      %   is the sum of the outputs of obj and other. If same_input = true, the
      %   returned DrakeFunction has an input of the same size as that of obj
      %   and other. If same_input = false, the returned DrakeFunction has an
      %   input whose length is the sum of those of the inputs of obj and other
      %
      % fcn = plus(obj,other) is equivalent to plus(obj,other,false)
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
        fcn = compose(Sum(obj.dim_output,2),concatenate(obj,other,same_input));
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
      fcn = compose(ConstantMultiple(obj.dim_output, -1),obj);
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
      fcn = compose(ConstantMultiple(fcn_orig.dim_input,value),fcn_orig);
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
      if n < 2 
        fcn = obj;
        return;
      end
      more_obj = repmat({obj},1,n-1);
      fcn = concatenate(obj,more_obj{:});
    end

    function fcn = addInputs(obj,n_additional_inputs,append)
      % fcn = appendInputs(obj,n_additional_inputs,append) returns a DrakeFunction that
      % is the same as obj, but with unused inputs added. These inputs are
      % appended or prepended based on the value of 'append'.
      %
      % @param obj
      % @param n_additional_inputs
      % @param append -- Logical indicating whether to add the inputs after (T)
      %                  or before (F) the current inputs. 
      %                  Optional. @default True 
      if nargin < 3, append = true; end
      dummy_fcn = drakeFunction.Zeros(0, n_additional_inputs);
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
                assert(s.subs{1}.frame.dim,obj.dim_input);
                x = double(s.subs{1});
              elseif isnumeric(s.subs{1})
                sizecheck(s.subs{1},[obj.dim_input,1]);
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

    function dim_input = getNumInputs(obj)
      dim_input = obj.dim_input;
    end

    function dim_output = getNumOutputs(obj)
      dim_output = obj.dim_output;
    end
  end
end

classdef Concatenated < drakeFunction.DrakeFunction
  % DrakeFunction representing the concatenation of n functions
  %
  % Implements either
  % \f[
  % f(x) = 
  % \begin{bmatrix}
  %   f_1(x_1) \\
  %   f_2(x_2) \\
  %   \vdots \\
  %   f_n(x_n)
  % \end{bmatrix},\;
  % \frac{df}{dx} = 
  % \begin{bmatrix}
  %   \frac{d f_1}{d x_1} &                     &         & 0                  \\
  %                       & \frac{d f_2}{d x_2} &         &                    \\
  %                       &                     & \ddots  &                    \\
  %   0                   &                     &         & \frac{d f_n}{d x_n} 
  % \end{bmatrix}
  % \f]
  %
  % where \f$x = (x_1, x_2, \dots, x_n)^\prime\f$ or 
  %
  % \f[
  % f(x) = 
  % \begin{bmatrix}
  %   f_1(x) \\
  %   f_2(x) \\
  %   \vdots \\
  %   f_n(x)
  % \end{bmatrix},\;
  % \frac{df}{dx} = 
  % \begin{bmatrix}
  %   \frac{d f_1}{d x} \\
  %   \frac{d f_2}{d x} \\
  %   \vdots            \\
  %   \frac{d f_n}{d x} 
  % \end{bmatrix}
  % \f]
  properties (SetAccess = immutable)
    contained_functions     % Cell array of DrakeFunction objects
    n_contained_functions   % Number of elements in contained_functions
    same_input              % Logical scalar indicating whether all of
                            % the contained function share the same input
    index_map               % Cell array of indices into the input vector
  end

  methods
    function obj = Concatenated(fcns,same_input)
      % obj = Concatenated(fcns, same_input) returns a DrakeFunction
      %   representing the concatenation of a given set of
      %   DrakeFunctions. If same_input = true, the input to the
      %   concatenated function is passed to all of the component
      %   functions. Otherwise, the input to the concatenated function
      %   is split and distributed to the component functions.
      %
      % obj = Concatenated(fcns) is the same as Concatenated(fcns,true).
      %
      % @param fcns         -- Cell array of DrakeFunction objects
      % @param same_input   -- Logical scalar
      if nargin < 2, same_input = false; end
      typecheck(fcns,'cell');
      assert(all(cellfun(@(arg)isa(arg,'drakeFunction.DrakeFunction'), fcns)));

      dim_input = drakeFunction.Concatenated.computeNumberOfInputs(fcns,same_input);
      dim_output = drakeFunction.Concatenated.computeNumberOfOutputs(fcns);

      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);

      obj.index_map = cell(numel(fcns),1);
      offset = 0;
      for i = 1:numel(fcns)
        obj.index_map{i} = offset + (1:fcns{i}.dim_input);
        if ~same_input
          offset = offset + fcns{i}.dim_input;
        end
      end
      obj.contained_functions = fcns;
      obj.n_contained_functions = numel(fcns);
      obj.same_input = same_input;
      obj = obj.setSparsityPattern();
    end

    function [f,df,ddf] = eval(obj,x)
      compute_second_derivatives = (nargout > 2);
      if compute_second_derivatives
        [f_cell,df_cell,ddf_cell] = evalContainedFunctions(obj,x);
        [f,df,ddf] = combineOutputs(obj,f_cell,df_cell,ddf_cell);
      else
        [f_cell,df_cell] = evalContainedFunctions(obj,x);
        [f,df] = combineOutputs(obj,f_cell,df_cell);
      end
    end

    function obj = setSparsityPattern(obj)
      % [iCfun, jCvar] = getSparsityPattern(obj) returns the row and
      %   column indices of the potentially non-zero elements of this
      %   function's Jacobian.
      %
      % @param obj      -- drakeFunction.Concatenated object 
      % 
      % @retval iCfun   -- Vector of row indices of the non-zeros
      % @retval jCvar   -- Vector of column indices of the non-zeros
      f_cell = cell(obj.n_contained_functions,1);
      df_cell = cell(obj.n_contained_functions,1);
      for i = 1:obj.n_contained_functions
        f_cell{i} = NaN(obj.contained_functions{i}.getNumOutputs(),1);
        df_cell{i} = ones(obj.contained_functions{i}.getNumOutputs(), ...
                          obj.contained_functions{i}.getNumInputs());
      end
      [~,df] = combineOutputs(obj,f_cell,df_cell);
      [obj.iCfun, obj.jCvar] = find(df);
    end
  end

  methods (Access = private)
    function [f_cell,df_cell] = evalContainedFunctions(obj,x)
      % [f_cell,df_cell] = evalContainedFunctions(obj,x) returns the
      % function values and gradients for each of the component
      % functions
      % 
      % @param obj        -- drakeFunction.Concatenated object
      % @param x          -- Input vector
      %
      % @retval f_cell    -- Cell array of function values for each
      %                      component function
      % @retval df_cell   -- Cell array of Jacobians for each component
      %                      function
      compute_second_derivatives = (nargout > 2);
      f_cell = cell(obj.n_contained_functions, 1);
      df_cell = cell(obj.n_contained_functions, 1);
      if compute_second_derivatives
        ddf_cell = cell(obj.n_contained_functions, 1);
      end
      contained_functions_local = obj.contained_functions;
      if compute_second_derivatives
        for i = 1:obj.n_contained_functions
          [f_cell{i},df_cell{i},ddf_cell{i}] = eval(contained_functions_local{i},x(obj.index_map{i}));
        end
      else
        for i = 1:obj.n_contained_functions
          [f_cell{i},df_cell{i}] = eval(contained_functions_local{i},x(obj.index_map{i}));
        end
      end
    end

    function [f,df,ddf] = combineOutputs(obj,f_cell,df_cell,ddf_cell)
      compute_second_derivatives = (nargout > 2);
      f = vertcat(f_cell{:});
      if obj.same_input
        df = vertcat(df_cell{:});
      else
        df = blkdiag(df_cell{:});
      end
      if compute_second_derivatives
        if obj.same_input
          ddf = vertcat(ddf_cell{:});
        else
          error('Really hoping we don''t get here')
        end
      end
    end
  end

  methods (Static)
    function dim_input = computeNumberOfInputs(fcns, same_input)
      if nargin < 2, same_input = false; end
      if same_input
        dim_input = fcns{1}.dim_input;
        % Check that all elements of fcns have the same number of inputs
        assert(all(cellfun(@(fcn) fcn.dim_input == dim_input, fcns(2:end))), ...
          'Drake:DrakeFunction:NInputsDoNotMatch', ...
          ['If ''same_input'' is set to true, all functions must ' ...
           'have inputs of the same size']);
      else
        dim_input = sum(cellfun(@(fcn) fcn.dim_input, fcns));
      end
    end

    function dim_output = computeNumberOfOutputs(fcns)
      dim_output = sum(cellfun(@(fcn) fcn.dim_output, fcns));
    end

  end
end

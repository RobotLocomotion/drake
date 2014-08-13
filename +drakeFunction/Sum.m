classdef Sum < drakeFunction.Container
  methods
    function obj = Sum(terms,same_input)
      % obj = SumFunction(terms,same_input) constructs a SumFunction
      % object.
      if nargin < 2, same_input = false; end
      obj = obj@drakeFunction.Container(terms,same_input,true);
    end
    function [f,df] = combineOutputs(obj,f_cell,df_cell)
      f = sum(horzcat(f_cell{:}),2);
      if obj.same_input
        df = sum(cat(3,df_cell{:}),3);
      else
        df = horzcat(df_cell{:});
      end
    end
  end
end

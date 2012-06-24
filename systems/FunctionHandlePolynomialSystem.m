classdef FunctionHandlePolynomialSystem < PolynomialSystem
  
  properties (SetAccess=private, GetAccess=private)
    dynamics_handle
    update_handle
    output_handle
    state_constraints_handle
    mass_matrix_handle
  end

  methods
    function obj = FunctionHandlePolynomialSystem()
      
    end
  end
end

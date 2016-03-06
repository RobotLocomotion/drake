classdef PolytopicPolynomialSystem < PolytopicSystem

  methods
    function obj = PolytopicPolynomialSystem(A,b,subsys)
      obj=obj@PolytopicSystem(A,b,subsys);
    end
  end
  
end


classdef LTISystem < AffineSystem %& TimeVaryingLinearSystem
  
  methods 
    function obj=LTISystem(Ac,Bc,Ad,Bd,C,D)
      % Constructor for LTI system.
      obj = obj@AffineSystem(Ac,Bc,[],Ad,Bd,[],C,D,[]);
    end

    function sys=feedback(sys1,sys2)
      sys = feedback@AffineSystem(sys1,sys2);
      sys = extractLTISystem(sys);
    end

    function sys=cascade(sys1,sys2)
      sys = cascade@AffineSystem(sys1,sys2);
      sys = extractLTISystem(sys);
    end
  end
end

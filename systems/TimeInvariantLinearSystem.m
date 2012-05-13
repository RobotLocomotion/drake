classdef TimeInvariantLinearSystem < AffineSystem %& TimeVaryingLinearSystem
  
  methods 
    function obj=TimeInvariantLinearSystem(Ac,Bc,Ad,Bd,C,D)
      % Constructor for LTI system.
      obj = obj@AffineSystem(Ac,Bc,[],Ad,Bd,[],C,D,[]);
    end
    
  end
end

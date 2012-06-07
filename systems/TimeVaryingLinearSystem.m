classdef TimeVaryingLinearSystem < TimeVaryingAffineSystem
% dynamics, update, output are linear in x and u, but not necessarily in t

  methods 
    function obj=TimeVaryingLinearSystem(Ac,Bc,Ad,Bd,C,D)
      obj = obj@TimeVaryingAffineSystem(Ac,Bc,[],Ad,Bd,[],C,D,[]);
    end

    function sys=feedback(sys1,sys2)
      sys = feedback@TimeVaryingAffineSystem(sys1,sys2);
      sys = extractLTVSystem(sys);
    end

    function sys=cascade(sys1,sys2)
      sys = cascade@TimeVaryingAffineSystem(sys1,sys2);
      sys = extractLTVSystem(sys);
    end
  end
end

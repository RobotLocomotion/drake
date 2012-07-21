classdef LinearSystem < AffineSystem 
  
  methods 
    function obj=LinearSystem(Ac,Bc,Ad,Bd,C,D)
      % Constructor for LTI system.
      obj = obj@AffineSystem(Ac,Bc,[],Ad,Bd,[],C,D,[]);
    end

    function sys=feedback(sys1,sys2)
      sys = feedback@AffineSystem(sys1,sys2);
      if isa(sys2,'LinearSystem')
        sys = extractLinearSystem(sys);
      end
    end

    function sys=cascade(sys1,sys2)
      sys = cascade@AffineSystem(sys1,sys2);
      if isa(sys2,'LinearSystem')
        sys = extractLinearSystem(sys);
      end
    end
  end
end

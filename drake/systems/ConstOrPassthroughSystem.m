classdef ConstOrPassthroughSystem < AffineSystem 
% Passes through a signal from input to output, optionally replacing some
% outputs with constant values.  This is useful, for instance, when
% emulating saturated signals.
% 
% To allow signals to pass through, set the const_y value to NaN.  
  
  methods 
    function obj = ConstOrPassthroughSystem(const_y, num_u)
      if (isa(const_y,'Point'))
        outframe = const_y.getFrame();
        const_y = double(const_y);
      else 
        outframe=[];
      end
      typecheck(const_y,'double');
      if (~isvector(const_y)) error('const_y must be a vector'); end
      n=length(const_y); 
      if (nargin<2) num_u = n; end;
      ind=~isnan(const_y);
      D=zeros(n,num_u);  D(~ind,~ind)=eye(sum(~ind));
      y0=zeros(n,1); y0(ind)=const_y(ind);
      obj = obj@AffineSystem([],[],[],[],[],[],[],D,y0);
      if ~isempty(outframe)
        obj = setOutputFrame(obj,outframe);
      end
    end
  end
end

classdef (InferiorClasses = {?Trajectory}) PolynomialWTimeVaryingCoefficients 
  % defines a polynomial (represented by an msspoly) with time-varying
  % coefficients (represented as a trajectory)
  
  properties
    poly;
    coeftraj;
    coefpoly;
  end

  methods
    function obj= PolynomialWTimeVaryingCoefficients(poly,coefpoly,coeftraj)
      % usage:
      %  PolynomialWTimeVaryingCoefficients(poly)
      % or 
      %  PolynomialWTimeVaryingCoefficients(poly,coefpoly,coeftraj)
      %   (which works like subs(poly,coefpoly,coeftraj)
      %
      % @param poly an msspoly (with constant coefficients) that will
      % be tranformed into a poly with time-varying coefficients (as
      % constant trajectories)
      %
      % @param coefpoly a free msspoly which represents polynomial
      % variables in poly that should be treated as if they were
      % coefficients
      % @param coeftraj trajectory objects which describe the time-varying
      % coefficients matching coefpoly
      
      
      typecheck(poly,'msspoly');
      
      if (nargin<2)
        [x,p,M]=decomp(poly);
        [i,j,s]=find(M);
      
        obj.coefpoly = PolynomialWTimeVaryingCoefficients.getUniquePoly(poly,[numel(s) 1]);
        obj.coeftraj = ConstantTrajectory(s);

        % this is just a bad way to do what I'd like to do: sparse(i,j,obj.coefpoly)
        Mpoly=msspoly(M);
        for k=1:length(i)
          Mpoly(i(k),j(k))=obj.coefpoly(k);
        end
      
        obj.poly = Mpoly*recomp(x,p);
      else
        if (nargin<3) error('must supply 1 argument or 3 arguments'); end
        typecheck(coefpoly,'msspoly');
        typecheck(coeftraj,'Trajectory'); 
        if (~iscolumn(coeftraj)) error('arg 3 must be a column vector trajectory'); end
        
        [x,p,M]=decomp(poly);
        b=match(coefpoly,x);
        polydims=b==0; trajdims=b>0;
        
        [ppoly,i,j]=unique(p(:,polydims),'rows');
        repmat(x(polydims)',size(ppoly,1),1).^ppoly;
        keyboard;
        
        % unfortunately, I have to play with msspoly internals here (for now).  
        [ms,ns]=size(poly.s);
        
        obj.coefpoly = PolynomialWTimeVaryingCoefficients.getUniquePoly(poly,coefpoly,ms);  % make potentially too many coefficients (can clean up later)
        k=round((ns-3)/2);
        vs=poly.s(:,3:2+k);
        ds=poly.s(:,3+k:2+2*k);
        cs=poly.s(:,ns);

        [f,xn]=isfree(coefpoly);
        if ~f, error('2nd argument must be a free msspoly column vector'); end
        if size(xn,2)~=1, error('input 2 must be a column'); end
                
        ee=mss_match(xn,vs);

        na=length(xn);
        if na<1, q=p; return; end
        [f,x]=issimple(b);
        if ~f, error('3rd argument must be a simple msspoly column vector'); end
        if size(x,1)~=size(xn,1), error('unequal number of variables'); end

        
        
        Cs=ones(size(vs));
        ee=mss_match(xn,vs);
        eee=(ee>0);
        eeee=ee(eee);
        Cs(eee)=x(eeee,2);
        vs(eee)=x(eeee,1);
        cs=cs.*prod(Cs.^ds,2);
        q=msspoly(p.m,p.n,[p.s(:,1:2) vs ds cs]);
        
        
        if (deg(poly,coefpoly)>1) error('coefpoly appears in poly with deg > 1'); end
        b=match(coefpoly,x);
        ind = (b==0);
        

        x=x(b==0);
        p=p(:,b==0);
        
        
        % my inverse of decomp
        % ps=reshape(M*prod(repmat(x',size(p,1),1).^p,2),size(pm));
        
        % my recomp
        [mp,np]=size(p);
        obj.poly=msspoly(mp,1,[(1:mp)' ones(mp,1) (p>0)*diag(xn) p ones(mp,1)]);

        obj.coefpoly = PolynomialWTimeVaryingCoefficients.getUniquePoly(poly,coefpoly,numel(s));
      end
      
    end
    
    function poly = getPoly(obj,t)
      poly = subs(obj.poly,obj.coefpoly,obj.coeftraj.eval(t));
    end
    
    function a = ctranspose(a)
      a.poly = ctranspose(a.poly);
    end
    
    function c = plus(a,b)
      error('not implemented yet');
    end
    
    function c = mtimes(a,b)
      aistraj = isa(a,'Trajectory') || isnumeric(a); aispoly = isa(a,'PolynomialWTimeVaryingCoefficients');       
      bistraj = isa(b,'Trajectory') || isnumeric(b); bispoly = isa(b,'PolynomialWTimeVaryingCoefficients');       
      
      if (bispoly)
        if (aistraj)
          acoefpoly=PolynomialWTimeVaryingCoefficients.getUniquePoly(b,size(a));
          cpoly = acoefpoly*b.poly;
          c=PolynomialWTimeVaryingCoefficients(cpoly,[b.coefpoly;acoefpoly(:)],[b.coeftraj;a(:)]);
        else
          error('not implemented yet');
        end
      else
        error('not implemented yet');
      end
    end
  end
  
  methods (Static=true) % todo: make it private
    function p=getUniquePoly(varargin)
      c=97;
      s=varargin{end};
      typecheck(s,'double');
      sizecheck(s,2);
      for i=1:length(varargin)-1
        safe = false;
        while (~safe)
          p=msspoly(char(c),prod(s));
          switch class(varargin{i})
            case 'PolynomialWTimeVaryingCoefficients'
              if ~any(match(p,decomp(varargin{i}.poly))) && ~any(match(p,varargin{i}.coefpoly))
                safe=true;
                continue;  % then this variable choice is safe
              end
            case 'msspoly'
              if ~any(match(p,decomp(varargin{i})))
                safe=true;
                continue;
              end
          end
          c=c+1;  % otherwise, go to the next letter
        end
      end
      p=reshape(p,s(1),s(2));
    end    
  end 
end

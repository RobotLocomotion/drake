classdef (InferiorClasses = {?Trajectory,?FunctionHandleTrajectory,?PPTrajectory,?ODESolTrajectory}) PolynomialWTimeVaryingCoefficients
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
        if (deg(poly,coefpoly)<1) obj = PolynomialWTimeVaryingCoefficients(poly); end

%        [x,p,M]=decomp(poly);
%        b=match(coefpoly,x);
%        polydims=b==0; trajdims=b>0;

%        [ppoly,i,j]=unique(p(:,polydims),'rows');
%        repmat(x(polydims)',size(ppoly,1),1).^ppoly;

        % unfortunately, I have to play with msspoly internals here (for now).
        [m,n]=size(poly);
        s=gets(poly);
        [ms,ns]=size(s);

        k=round((ns-3)/2);
        vs=s(:,3:2+k);
        ds=s(:,3+k:2+2*k);
        cs=s(:,ns);

        %% first merge the coefficient trajectories associated with the coefpolys
        [f,xn]=isfree(coefpoly);
        if ~f, error('2nd argument must be a free msspoly column vector'); end
        if size(xn,2)~=1, error('input 2 must be a column'); end

        Cs=ConstantTrajectory(ones(size(vs)));
        ee=mss_match(xn,vs);
        Cs(ee>0)=coeftraj(ee(ee>0));
        cs=cs.*prod(Cs.^ds,2);
        vs=vs(:,all(ee==0));
        ds=ds(:,all(ee==0));

        %% now collapse duplicate rows (summing the coefficient trajectories)
        a=[s(:,1:2) vs ds];
        [new_a,i,j]=unique(a,'rows');

        new_cs = ConstantTrajectory(zeros(length(i),1));
        for k=1:length(cs)
          new_cs(j(k))=new_cs(j(k))+cs(k);
        end

        % now replace coefficients with coefpolys and I'm done.
        obj.coefpoly = PolynomialWTimeVaryingCoefficients.getUniquePoly(poly,coefpoly,[length(i) 1]);  % make potentially too many coefficients (can clean up later)
        [~,new_v] = isfree(obj.coefpoly);
        obj.coeftraj = new_cs;
        obj.poly=msspoly(m,n,[s(i,1:2),vs(i,:),new_v,ds(i,:),ones(length(i),2)]);
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

    function s = size(varargin)
      s = size(varargin{1}.poly,varargin{2:end});
    end

    function c = mtimes(a,b)
      aistraj = isa(a,'Trajectory') || isnumeric(a); aispoly = isa(a,'PolynomialWTimeVaryingCoefficients');
      bistraj = isa(b,'Trajectory') || isnumeric(b); bispoly = isa(b,'PolynomialWTimeVaryingCoefficients');

      if (bispoly)
        if (aistraj)
          acoefpoly=PolynomialWTimeVaryingCoefficients.getUniquePoly(b,size(a));
          cpoly = acoefpoly*b.poly;
          c=PolynomialWTimeVaryingCoefficients(cpoly,[b.coefpoly;acoefpoly(:)],[b.coeftraj;a(:)]);
        elseif (aispoly)
          % change variables to make sure there are no symbol crashes
          acoefpoly=PolynomialWTimeVaryingCoefficients.getUniquePoly(b,size(a.coefpoly));
          cpoly = subs(a.poly,a.coefpoly,acoefpoly)*b.poly;
          c=PolynomialWTimeVaryingCoefficients(cpoly,[b.coefpoly;acoefpoly(:)],[b.coeftraj;a.coeftraj]);
        else
          error('not implemented yet');
        end
      elseif (bistraj && aispoly)
          bcoefpoly=PolynomialWTimeVaryingCoefficients.getUniquePoly(a,size(b));
          cpoly = a.poly*bcoefpoly;
          c=PolynomialWTimeVaryingCoefficients(cpoly,[bcoefpoly(:);a.coefpoly],[b(:);a.coeftraj]);
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

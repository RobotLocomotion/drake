function [varargout]=qlcon2vert(x0,varargin)
%A quicker version of lcon2vert that will skip some intensive steps when a
%point x0 in the relative interior of the polyhedron is known a priori.
%
%   [V,nr,nre]=qlcon2vert(x0, A,b,Aeq,beq,TOL)
%
%Here x0 is the known relative interior point. The rest of the input/output
%arguments are as in LCON2VERT.
%
%In the interest of speed, QLCON2VERT will perform minimal error checking,
%shifting the burden to the user of verifying that the input data has
%appropriate properties. In particular,  it will not try to verify that x0
%is in fact in the relative interior. By default also, qlcon2vert will skip the 
%boundedness test, and presume the  user knows the polyhedron to be bounded already.
%However, you can re-activate the boundedness test by calling with the syntax,
%
%  [V,nr,nre]=qlcon2vert(x0, A,b,Aeq,beq,TOL,1)


if nargin<=3

    varargin(3:6)={[],[],[],0};
    
end

x0=x0(:);

[A,b,Aeq,beq]=deal(varargin{1:4});

b=b-A*x0;
if ~isempty(beq)
    beq(:)=0;
end

varargin{2}=b;
varargin{4}=beq;

[varargout{1:max(nargout,1)}] = lcon2vert(varargin{:});

 V=varargout{1};

 V=bsxfun(@plus,x0.',V); %undo coordinate translation

varargout{1}=V;

function y=getProjection(x,f,x0,plotdims,options)
% return points on the (first) level set f(x)==1 surrounding x0 projected
% onto the plane plotdims
%
% @param x a simple msspoly defining the coordinates
% @param f an msspoly of the function you want to plot
% @param x0 the fixed point around which the function will be plotted.  @
% default 0
% @param plotdims the dimensions on which to project.  @default [1 2]
% @option num_samples number of samples to produce @default 100

if (nargin<5) options=struct(); end
if (~isfield(options,'tol')) options.tol = 2e-3; end % default tolerance of fplot
if (~isfield(options,'method')) options.method = 'hull'; end
if (~isfield(options,'num_samples')) options.num_samples = 100; end

typecheck(x,'msspoly'); 
if ~issimple(x) error('x should be a simple msspoly'); end
typecheck(f,'msspoly');
sizecheck(f,1);

if (deg(f,x)>2)
  error('Only projections of quadratics supported.');
end
% put in the form x'Qx + 2c'x + b
c = double(subs(diff(f,x),x,0*x)/2)';
Q = double(diff(diff(f,x)',x))/2;
b = double(subs(f,x,0*x));
  
d = - Q\c;
v = double(subs(f,x,d));
if v > 1, y = []; return; end
Q = Q/(1-v);
% First, construct a set of directions:
N = options.num_samples;
X = zeros(length(x0),N);
if length(plotdims) == 2
  thetas = linspace(-pi,pi,N);
  X(plotdims,:) = [cos(thetas); sin(thetas)];
else
  X(plotdims,:) = randn(length(plotdims),N);
end
% For each direction, max. f'x subj. to x'Sx <= 1 is equivalent to
%                     max. f'x subj. to x'Sx == 1 ''
%                     max. f'S^(-1/2)x  subj. to y'y == 1
extrm = (Q\X)./repmat(sqrt(sum(X.*(Q\X))),length(x0),1);
y = repmat(d(plotdims),1,N) + extrm(plotdims,:);

end

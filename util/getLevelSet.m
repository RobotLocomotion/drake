function y=getLevelSet(V,x0,options)

typecheck(V,'msspoly');
x = decomp(V);
if (nargin<2) x0=zeros(size(x,1),1); end

if (length(x) ~= 2) error('not supported yet'); end 
if (double(subs(V,x,x0))>=1) error('x0 is not in the one sub level-set of V'); end

% assume star convexity (about x0).

r = msspoly('r',1);
N=50;
th = linspace(0,pi,N);
circle=[cos(th);sin(th)];
y=zeros(2,2*N);  % pre-allocate

% todo:  consider balancing coordinates based on the quadratic terms in V

for i=1:N
  [a,p,M] = decomp(subss(1-V,x,r*circle(:,i)));
  c(max(p)+1-p)=M;
  z=roots(c);  % roots of poly 1-V along the line defined by th(i)
  % just keep the roots that bracket 0
  y(:,i) = min(z(find(z>=0)))*circle(:,i);
  y(:,N+i) = min(z(find(z<=0)))*circle(:,i);
end





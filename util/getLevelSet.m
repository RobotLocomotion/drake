function y=getLevelSet(V,x0,options)

typecheck(V,'msspoly');
x = decomp(V);
if (nargin<2 || isempty(x0)) x0=zeros(size(x,1),1); end
if (nargin<3) options=struct(); end
if (~isfield(options,'tol')) options.tol = 2e-3; end % default tolerance of fplot

if (double(subs(V,x,x0))>1) 
  error('x0 is not in the one sub level-set of V'); 
end

if (deg(V,x)<=2)  % interrogate the quadratic level-set
  H = doubleSafe(0.5*diff(diff(V,x)',x));
  b = -0.5*(H\doubleSafe(subs(diff(V,x),x,0*x)'));
  
  n=length(x);
  K=100;
  if (n==2)  % produce them in a nice order, suitable for plotting
    th=linspace(0,2*pi,K);
    X = [sin(th);cos(th)];
  else
    X = randn(n,K);
  end
  X = X./repmat(sqrt(sum(X.^2,1)),n,1);

  y = repmat(b,1,K) + (H/(doubleSafe(1-subs(V,x,b))))^(-1/2)*X;

else % do the more general thing

  if (length(x) ~= 2) error('not supported yet'); end 

  % assume star convexity (about x0).

  r = msspoly('r',1);
  [theta,r]=fplot(@getRadius,[0 pi],options.tol);
  y=[repmat(r(:,1),1,2).*[cos(theta),sin(theta)]; repmat(r(:,2),1,2).*[cos(theta),sin(theta)]]';

end


function y=getRadius(theta)  % needs to be vectorized
  circ = [cos(theta);sin(theta)];
  for i=1:length(theta)
    [a,p,M] = decomp(subss(1-V,x,r*circ(:,i)));
    c(max(p)+1-p)=M;
    z=roots(c);  % roots of poly 1-V along the line defined by th(i)
    z=z(find(~imag(z)));  % only keep the real roots

    % just keep the roots that bracket 0
    a=min(z(find(z>0))); if (isempty(a)) a=nan; end
    y(i,1) = a;
    a=max(z(find(z<0))); if (isempty(a)) a=nan; end
    y(i,2) = a;
  end
end



end


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

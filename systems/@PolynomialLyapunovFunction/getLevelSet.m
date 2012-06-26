function y=getLevelSet(V,x0,options)
% return points on the level set V(x)==1
% @param x0 the fixed point around which the Lyapunov function is
% constructed
% @option num_samples number of samples to produce @default 100

if (nargin<3) options=struct(); end
if ~isfield(options,'num_samples') options.num_samples = 100; end

x = V.getFrame.poly;

if (nargin<3) options=struct(); end
if (~isfield(options,'tol')) options.tol = 2e-3; end % default tolerance of fplot

if (deg(V.Vpoly,x)<=2)  % interrogate the quadratic level-set
  % note: don't need (or use) x0 in here
  
  H = doubleSafe(0.5*diff(diff(V.Vpoly,x)',x));
  b = -0.5*(H\doubleSafe(subs(diff(V.Vpoly,x),x,0*x)'));
  
  n=length(x);
  K=options.num_samples;
  if (n==2)  % produce them in a nice order, suitable for plotting
    th=linspace(0,2*pi,K);
    X = [sin(th);cos(th)];
  else
    X = randn(n,K);
    X = X./repmat(sqrt(sum(X.^2,1)),n,1);
  end

  y = repmat(b,1,K) + (H/(doubleSafe(1-subs(V.Vpoly,x,b))))^(-1/2)*X;
else % do the more general thing

  if (length(x) ~= 2) error('not supported yet'); end
  if (nargin<2 || isempty(x0)) x0=zeros(size(x,1),1); end

  pV = V.Vpoly;
  
  % assume star convexity (about x0).
  if (double(subs(pV,x,x0))>1)
    error('x0 is not in the one sub level-set of V');
  end
  
  pV=subss(pV,x,x+x0);  % move to origin


  y = repmat(x0,1,options.num_samples)+getRadii(linspace(-pi,pi,options.num_samples))';
  %  r = msspoly('r',1);
  %   [theta,r]=fplot(@getRadius,[0 pi],options.tol);
  %   y=repmat(x0,1,2*size(theta,1))+[repmat(r(:,1),1,2).*[cos(theta),sin(theta)]; repmat(r(:,2),1,2).*[cos(theta),sin(theta)]]';

end

if (any(imag(y(:)))) 
  error('something is wrong.  i got imaginary outputs'); 
end

% Assumes that the function is radially monotonic.  This could
% break things later.
function y=getRadii(thetas)  % needs to be vectorized
    rU = ones(size(thetas));
    rL = zeros(size(thetas));
    CS = [cos(thetas); sin(thetas)];
    evaluate = @(r) double(msubs(pV,x,repmat(r,2,1).*CS));
    msk = evaluate(rU) < 1;
    while any(msk)
        rU(msk) = 2*rU(msk);
        msk = evaluate(rU) < 1;
    end
    
    while (rU-rL) > 0.0001*(rU+rL) 
        r = (rU+rL)/2;
        msk = evaluate(r) < 1;
        rL(msk) = r(msk);
        rU(~msk) = r(~msk);
    end
    
    y = (repmat(r,2,1).*CS)';
end
function y=getRadius(theta)  % needs to be vectorized
  circ = [cos(theta);sin(theta)];
  for i=1:length(theta)
    [a,p,M] = decomp(subss(1-pV,x,r*circ(:,i)));
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

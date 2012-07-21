function vol=getLevelSetVolume(x,f)

typecheck(x,'msspoly');
if ~issimple(x), error('x must be simple'); end
typecheck(f,'msspoly');
sizecheck(f,1);

if (deg(f,x)<=2)  % interrogate the quadratic level-set
  % note: don't need (or use) x0 in here
  
  H = doubleSafe(0.5*diff(diff(f,x)',x));
  b = -0.5*(H\doubleSafe(subs(diff(f,x),x,0*x)'));
  
  c=1;  % the one level set
  n=length(x);
  vol = 2*((pi*c)^(n/2))/(n*gamma(n/2)*sqrt(det(H)));
  % from http://www.mathkb.com/Uwe/Forum.aspx/math/24356/how-to-find-the-volume-of-a-quadratic-form
  
else % do the more general thing

  error('not implemented yet for deg(f)>2');
  % todo:  estimate volume with largest contained ellipse
  
end

end


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

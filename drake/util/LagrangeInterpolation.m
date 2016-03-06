function pp = LagrangeInterpolation(t,y,dim)
% returns a degenerated PPTrajectory (in the sense that there's only one break) in [0,2] interval
% t, the interpolating points,
% y, values of the interpolating points, can be a vector or a matrix
% dim, the dimension of the pp desired

s = size(y);  n = prod(s(1:end-1));

if (nargin<3) 
  dim=n; 
else
  if (prod(dim)~=n) error('dim does not match size of y'); end
end
if (length(size(y))~=2) 
  y = reshape(y,n,[]);
end
if (size(y,2)~=length(t)) error('last dimension of y should be the same size as t'); end


for i=1:(dim) % can vectorize but not really necessary, as it is called only once in each traj optimization
coefs=polyfit(t,y(i,:)',length(t)-1); % polyfit has native warning message that's helpful for diagnosing Runge's phenomenon, but polyfit() only takes 1D vector input. 
% this is also a minor reason why this loop is not vectorized
coefs_shift=poly(roots(coefs)+1); % shift the polynomial to the right by 1 unit
coefs_shift_scale=coefs_shift*(coefs(1)/coefs_shift(1)); % poly() always return the scaled polynomial with highest power coefficient being 1, need to scale back 
coefs_all(i,:) = coefs_shift_scale;
end
pp=mkpp([0,2],coefs_all,dim); 
end
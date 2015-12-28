function pp = LagrangeInterpolation(t,y,dim)
% returns a degenerated PPTrajectory (in the sense that there's only one break) in [0,2] interval
% t, the interpolating points
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


for i=1:(dim)
coefs=polyfit(t,y(i,:)',length(t)-1);
coefs_shift=poly(roots(coefs)+1); % shift the polynomial to the right by 1 unit
coefs_shift_scale=coefs_shift*(coefs(1)/coefs_shift(1));
coefs_all(i,:) = coefs_shift_scale;
end
size(coefs_all);

pp=mkpp([0,2],coefs_all,dim); 
end
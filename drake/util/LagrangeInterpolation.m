function pp = LagrangeInterpolation(t,y,dim)
% it is tempting to interpolate at the physical time domain nodes, but as expected, with high N number, Runge phenomenon is observed. 
% to avoid Runge phenomenon, do the Lagrange interpolating at tau. The shifting and scaling will be taken care of later.

s = size(y);  n = prod(s(1:end-1));

if (nargin<5) 
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
coefs_shift=poly(roots(coefs)+1);
coefs_shift_more=coefs_shift*(coefs(1)/coefs_shift(1));
coefs_all(i,:) = coefs_shift_more;
end
size(coefs_all);

pp=mkpp([0,2],coefs_all,dim); 
end
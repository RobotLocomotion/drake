function [lin_params,coeff_mat,offsets] = linearParameters(expr,params)
% Given an expression (expr) and a set of parameters (params) computes
% a set of lumped parameters (lin_params), coefficients (coeff_mat), and
% offsets such that
%
%   expr = coeffs*lin_params + offsets
%
if isa(expr,'TrigPoly')
  q = getVar(expr);
  s = getSin(expr);
  c = getCos(expr);
  qt = TrigPoly(q,s,c);
  isTrigPoly = true;
  expr = getmsspoly(expr);
else
  isTrigPoly = false;
end
non_params = decomp(expr,params);
[a,b,coeff_mat]=decomp(expr,non_params);
coeff = msspoly(ones(size(b,1),1));
deg_zero_ind=-1;
for i=1:size(b,1)
  % would use prod(a'.^b(i,:)) if msspoly implemented prod (bug 1712)
  for k=find(b(i,:))
    coeff(i) = coeff(i).*(a(k)^b(i,k));
  end
  if (deg(coeff(i))==0)
    if (deg_zero_ind>0), error('should only get a single degree zero term'); end
    deg_zero_ind = i;
  end
end

if (deg_zero_ind>0)
  offsets = coeff_mat(:,deg_zero_ind)*double(coeff(deg_zero_ind));
  coeff_mat = coeff_mat(:,[1:deg_zero_ind-1,deg_zero_ind+1:end]);
  lin_params = coeff([1:deg_zero_ind-1,deg_zero_ind+1:end]);
else
  offsets = zeros(size(expr,1));
  lin_params = coeff;
end
if isTrigPoly
  coeff_mat = coeff_mat+0*qt(1); % There must be a better way to go from msspoly to TrigPoly . . .
  offsets = offsets+0*qt(1); % There must be a better way to go from msspoly to TrigPoly . . .
end
end
function [x,objval,exitflag] = gloptipoly(obj,options)

checkDependency('gloptipoly3');
checkDependency('sedumi');

vars = obj.decision_vars;
mpol('v',length(vars));
objective = msspoly2mpol(vars,v,clean(obj.poly_objective));

constraints=[];
if ~isempty(obj.poly_inequality_constraints)
  ineq = msspoly2mpol(vars,v,clean(obj.poly_inequality_constraints));
  constraints = vertcat(constraints,ineq<=0);
end
if ~isempty(obj.poly_equality_constraints)
  eq = msspoly2mpol(vars,v,clean(obj.poly_equality_constraints));
  constraints = vertcat(constraints,eq==0);
end
if ~isempty(obj.Ain)
  constraints = vertcat(constraints,Ain*v <= bin);
end
if ~isempty(obj.Aeq)
  constraints = vertcat(constraints,Aeq*v == beq);
end
if any(~isinf(obj.x_lb))
  ind = ~isinf(obj.x_lb);
  constraints = vertcat(constraints,v(ind) >= obj.x_lb(ind));
end
if any(~isinf(obj.x_ub))
  ind = ~isinf(obj.x_ub);
  constraints = vertcat(constraints,v(ind) <= obj.x_ub(ind));
end

global MMM; MMM.verbose = false;  % disable gloptipoly debug spews

exitflag = 0;
relaxation_order = 1;  % todo: make this an option
while (exitflag == 0)
  try 
    if isempty(constraints)
      prog = msdp(min(objective),relaxation_order);
    else
      prog = msdp(min(objective),constraints,relaxation_order);
    end
  catch ex
    if strcmp(ex.message,'Increase relaxation order')
      relaxation_order = relaxation_order+1
      continue;
    end
    rethrow(ex);
  end
    
  [exitflag,objval] = msol(prog);
  relaxation_order = relaxation_order+1
  % todo: make it an option whether we want to loop til convergence
end
if (exitflag<1)
  x = nan(obj.num_vars,1);
else
  x = double(v);  % note: this appears to return all solutions.  should I just take the first?
end

end

function pmpol = msspoly2mpol(xmss,xmpol,pmss)

[x,p,M] = decomp(pmss);
% decomp places the variables in a different order.
% we must sort xs to match this order.
[~,xn]  = isfree(x);  % extract ids of x.
[~,xmn] = isfree(xmss); % extract ids of xm.
perm = mss_match(xmn,xn); % compute permutation.
xmpol = xmpol(perm);

% want to use this (from decomp), but mpol doesn't have prod(x,dim):
%pmpol=reshape(M*prod(repmat(xmpol',size(p,1),1).^p,2),size(pmss));

% here is the unrolled version
pmpol=ones(size(p,1),1);
for i=1:size(p,2)
  pmpol = pmpol.*(repmat(xmpol(i,:)',size(p,1),1).^p(:,i));
end
pmpol = reshape(M*pmpol,size(pmss));


end
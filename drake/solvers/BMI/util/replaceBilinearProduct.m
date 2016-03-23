function exprX = replaceBilinearProduct(expr,w,W)
% if we have an expresion that involves linear and bilinear terms of w, then we replace
% the bilinear terms using the matrix W, W is supposed to be equal to w*w'
% @param expr  A spotless expression
% @param w     A simple vector of size nw x 1
% @param W     A matrix of size nw x nw
if(~issimple(w))
  error('w should be simple');
end
[vars,degrees,coefs,sizeExpr] = decomp(expr);
num_vars = length(vars);
if(num_vars > 0)
  nw = length(w);
  tril_W_mask = tril(ones(size(W)))~= 0;
  varsW = W(tril_W_mask);
  vars2w = match(w,vars); % vars(i) = w(vars2w(i))
  var_w_flag = vars2w ~= 0; % vars(var_w_flag) are those variables that appear in w
  degree_w_sum = sum(degrees.*bsxfun(@times,ones(size(degrees,1),1),var_w_flag'),2);
  if(any(degree_w_sum > 2))
    error('Only accepts linear or bilinear terms');
  end
  Wij_idx = reshape(cumsum(tril_W_mask(:)),nw,nw).*tril_W_mask;
  W_diag_idx = diag(reshape(1:nw*nw,nw,nw));
  Wij_idx_diag = Wij_idx(W_diag_idx);
  Wij_idx = Wij_idx+Wij_idx';
  Wij_idx(W_diag_idx) = Wij_idx_diag;
  degreesW = sparse(size(degrees,1),length(varsW));
  for i = 1:size(degrees,1)
    inds = find(degrees(i,:)~=0 & var_w_flag');
    if(length(inds) == 1 && degrees(i,inds) == 2)
      degrees(i,inds) = 0;
      degreesW(i,Wij_idx(vars2w(inds),vars2w(inds))) = 1;
    elseif(length(inds) == 2)
      degrees(i,inds) = 0;
      degreesW(i,Wij_idx(vars2w(inds(1)),vars2w(inds(2)))) = 1;
    end
  end
  exprX = recomp([vars;varsW],[degrees degreesW],coefs,sizeExpr);
else
  exprX = expr;
end
end

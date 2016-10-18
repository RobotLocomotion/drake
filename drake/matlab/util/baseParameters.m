function [beta, independent_idx] = baseParameters(W)
% Implements the method described in Appendix 5 of Kahlil and Dombre 2004.
[r,m] = size(W);
r_orig = r;
m_orig = m;
if isnumeric(W)
  [~,R,e]=qr(W,0);
  numerical_zero = r*eps*max(abs(diag(R)));
  independent_idx = abs(diag(R)) > numerical_zero;
  dependent_idx = ~independent_idx;
  b = sum(independent_idx);
  R1 = R(1:b,independent_idx);
  R2 = R(1:b,dependent_idx);
%   beta = R1\R2;
  beta = inv(R1)*R2;
  integer_entry_idx = abs(beta-round(beta)) < numerical_zero;
  beta(integer_entry_idx) = round(beta(integer_entry_idx));
  beta = [eye(b),beta];
  beta(:,[e(independent_idx),e(dependent_idx)]) = beta;
  beta_tmp = zeros(m);
  beta_tmp(e(independent_idx),:) = beta;
  beta_tmp(all(beta_tmp==0,2),:) = [];
  beta = beta_tmp; 
  independent_idx = sort(e(independent_idx));
%  independent_idx = e(independent_idx);
elseif isa(W,'msspoly')
  if r ~= 1
    W = reshape(W',[],1)';
  end
  vars = decomp(W);
  n_vars = length(vars);
  r = 10*m;
  data = rand(length(vars),r);
  W_data = reshape(msubs(W',vars,data),m,r*r_orig)';
  [beta, independent_idx] = baseParameters(W_data);
elseif isa(W,'TrigPoly')
  q = getVar(W);
  s = getSin(W);
  c = getCos(W);
  nq = length(q);
  vars = [];
  for i = 1:size(W,1)
    fprintf('i = %d\n',i);
    vars = mss_unique([vars;decomp(getmsspoly(W(i,:)),[q;s;c])]);
  end
  if r ~= 1
    W = reshape(W',[],1)';
  end
  r = 10*m; 
  q_data = rand(nq,r); s_data = sin(q_data); c_data = cos(q_data);

%   vars = decomp(getmsspoly(W),[q;s;c]);
  data = [q_data;s_data;c_data;rand(length(vars),r)];
  W_data = zeros(size(W,2),r); % [r x r_orig*m_orig]
  for i=1:size(W,2)
    fprintf('i = %d\n',i);
    W_data(i,:) = msubs(getmsspoly(W(i)),[q;s;c;vars],data); % [r x 1]
  end
  W_data = reshape(W_data,m,r*r_orig)'; % [m x r*r_orig]
  [beta, independent_idx] = baseParameters(W_data);
else
  error('baseParameters:invalidInput', ...
    'Objects of class %s are not valid inputs for getBaseParameters.',class(W));
end
end
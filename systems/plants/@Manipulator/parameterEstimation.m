function [phat,estimated_delay] = parameterEstimation(obj,data,options)
%
% Parameter estimation algorithm for manipulators
%
% Attempts to minimize the objective 
%  \[ \min_p  | H(q,p)qddot - C(q,qd,p) - B(q,qd,p)*u |_2^2 \]
% by extracting an affine representation using lumped-parameters and then
% running least-squares.
%
% Restrictions: 
%   H,C,and B must be trig/poly in q,qd and polynomial in p.
%   All params p must be lower bounded (general constraints are not
%   implemented yet, but would be easy if they are affine)
%   so far, I require full-state feedback
%
% Algorithm:
%   Step 1: Extract lumped-parameters 
%      Use TrigPoly + spotless to parse H,C,B and extract unique monomial
%      coefficients in p.
%   Step 2: Least-squares estimation of the lumped-parameters (more many
%      candidate unit delays)
%      Insert the data and do linear regression
%   Step 3: Geometric program to back out original parameters.
%
% @param obj must be a Manipulator which also implements the parameter
% interface (by deriving from the ParameterizedSystem class)
% @param data an instance of iddata (from the system identification 
% toolbox; see 'help iddata' for more info) containing the data to 
% be used for estimation.
%

typecheck(obj,'ParameterizedSystem'); % only works if the system also implements the parameters interface
[num_param_ineq,num_param_eq]=getNumParamConstraints(obj);
if num_param_ineq~=0 || num_param_eq~=0
  error('not supported yet');
end

if (getOutputFrame(obj)~=getStateFrame(obj))
  error('Only full-state feedback is implemented so far');
end

nq = obj.num_q;
nu = obj.num_u;
p_orig = double(getParams(obj));  % probably only for testing

%%   Step 1: Extract lumped-parameters 

q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
qt=TrigPoly(q,s,c);
qd=msspoly('qd',nq);
qdd=msspoly('qdd',nq);
u=msspoly('u',nu);
p=obj.getParamFrame.poly;
pobj = setParams(obj,p);
[H,C,B] = manipulatorDynamics(pobj,qt,qd);
err = H*qdd - C - B*u;

[a,b,M]=decomp(getmsspoly(err),[q;s;c;qd;qdd;u]);
coeff = msspoly(ones(size(b,1),1));
deg_zero_ind=-1;
for i=1:size(b,1)
  % would use prod(a'.^b(i,:)) if msspoly implemented prod (bug 1712)
  for k=find(b(i,:))
    coeff(i) = coeff(i).*(a(k)^b(i,k));
  end
  if (deg(coeff(i))==0) 
    if (deg_zero_ind>0) error('should only get a single degree zero term'); end
    deg_zero_ind = i;
  end
end

if (deg_zero_ind>0)
  Mb = M(:,deg_zero_ind)*double(coeff(deg_zero_ind));
  M = M(:,[1:deg_zero_ind-1,deg_zero_ind+1:end]);
  lp = coeff([1:deg_zero_ind-1,deg_zero_ind+1:end]);
else
  Mb = zeros(size(err,1));
  lp = coeff;
end

np = length(p_orig);
nlp = length(lp);
lp_orig = double(subs(lp,p,p_orig));
lumped_params = msspoly('lp',nlp);
% now err=M*lp+Mb and lperr=M*lumped_params+Mb;

%%   Step 2: Least-squares estimation

if (nargin>1)  % temporary hack so I can work on gp before finishing this
% populate A and b matrices from iddata
% todo: make the processing of q,qd,qdd more robust
Ts = get(data,'Ts');
t_data = get(data,'SamplingInstants')';
x_data = get(data,'OutputData')';
q_data = x_data(1:nq,:);
qd_data = x_data(nq+(1:nq),:);
u_data = get(data,'InputData')';

qdd_data = diff(qd_data,1,2)/Ts;
t_data = t_data(:,1:end-1);
q_data = q_data(:,1:end-1);
qd_data = qd_data(:,1:end-1);
u_data = u_data(:,1:end-1);
s_data = sin(q_data);
c_data = cos(q_data);

ndata = length(t_data);
M_data = reshape(msubs(M(:),[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])',nq*ndata,nlp);
Mb_data = reshape(msubs(Mb,[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])',nq*ndata,1);

lp_est = -M_data\Mb_data;
err_orig = M_data*lp_orig + Mb_data;

% just for debugging
clf;hold on;
for i=1:ndata
  [H,C,B] = manipulatorDynamics(obj,q_data(:,i),qd_data(:,i));
  err_check = H*qdd_data(:,i) - C - B*u_data(:,i);
  plot(t_data(:,i),err_check(1),'.');
  valuecheck(err_orig(i:ndata:nq*ndata),err_check);
end

err_est = M_data*lp_est + Mb_data;
sqerr_orig = sum(err_orig'*err_orig);
sqerr_est = sum(err_est'*err_est);

keyboard;

else
  lp_est = lp_orig;
end


%%   Step 3: Geometric program to back out original parameters.
%
% min | log(p) - log(p_orig) |_2^2 
%  s.t. forall i, lp_i = lp_est_i
%                 pmin <= p <= pman
%
% decision variables y_i, p_i = e^y_i, log(p_i) = y_i
%   allows me to write this as 
% min | y - log(p_orig) |_2^2 
%  s.t.   lp_i(p) = lp_est_i, 
%  s.t.   log(pmin) <= p <= log(pmax)
%  where lp_i is a monomial in p (with coeff = 1)
%  and taking the log of this constraint gives the equivalent constraint
%         A y = log(lp_est_i) where
%  A(i,j) is the power of the monomial in parameter j

A = sparse(nlp,np);
for i=1:nlp  % todo:  there must be a better way to do this with spotless
  [a,b,M] = decomp(lp(i));
  assert(isscalar(M));
  assert(M==1);
  for j=1:length(a)
    ind = find(match(a(j),p));
    A(i,ind) = b(j);
  end
end

irrelevant_params = find(~any(A));
if ~isempty(irrelevant_params)
  for i=1:length(irrelevant_params)
    warning(['Parameter ',getCoordinateName(getParamFrame(obj),irrelevant_params(i)),' does not impact the dynamics in any way.  Consider removing it from the parameter list']);
  end
%  A(:,irrelevant_params) = [];
%  p_map = find(any(A));
%else
%  p_map = 1:np;
end

[pmin,pmax] = getParamLimits(obj);
assert(all(pmin>=0));  % otherwise I'll have to subtract it out from the coordinates
%  not hard, just not implmented yet.
[log_p_est,~,exitflag] = quadprog(eye(np),-log(p_orig),[],[],A,log(lp_est),log(pmin),log(pmax),log(p_orig));

phat = Point(getParamFrame(obj),exp(log_p_est));

end

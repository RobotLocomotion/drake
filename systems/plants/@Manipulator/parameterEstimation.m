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
%   All params p must be lower bounded.
%   data must use the same variable names as the system coordinate frames
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

nq = obj.num_q;
nu = obj.num_u;
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

[a,b,M]=decomp(getmsspoly(err),[q;s;c;qd;qdd]);
coeff = msspoly(ones(size(b,1),1));
degzero=zeros(size(coeff));
for i=1:size(b,1)
  % would use prod(a'.^b(i,:)) if msspoly implemented prod (bug 1712)
  for k=find(b(i,:))
    coeff(i) = coeff(i).*(a(k)^b(i,k));
  end
  if (deg(coeff(i))==0) error('don''t expect this case, and would need to handle it explicitly'); end
end

% todo: do some extract work to find minimal coefficients here?

lumped_params = msspoly('lp',length(coeff));
% now err=M*coeff and lperr=M*lumped_params;



keyboard;
end

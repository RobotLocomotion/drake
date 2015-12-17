function [phat,estimated_delay] = parameterEstimation(obj,obj2,data,varargin)
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
% @param data an instance of iddata (from the system identification 
% toolbox; see 'help iddata' for more info) containing the data to 
% be used for estimation.
% 
% @option print_result determines if the function will print the results
% 0 = no print, 1 = print estimated and original, 2 = only print estimated
% 
% @option method determines which method to use for estimation
% 0 = no print, 1 = print estimated and original, 2 = only print estimated
% 

%% handle options
if (nargin>2 && isstruct(varargin{1})) options=varargin{1};
else options=struct(); end
if (~isfield(options,'print_result')) 
  options.print_result=0; 
end
if (~isfield(options,'method')) 
  options.method=0; 
end


%% Initialize
if (getOutputFrame(obj)~=getStateFrame(obj))
  error('Only full-state feedback is implemented so far');
end
checkDependency('spotless');

nq = obj.num_positions;
nu = obj.num_u;
p_orig = double(getParams(obj));  % probably only for testing
np = length(p_orig);

[pmin,pmax] = getParamLimits(obj);
assert(all(pmin>=0));  % otherwise I'll have to subtract it out from the coordinates
%  not hard, just not implmented yet.


%%   Step 1: Extract lumped-parameters 
% Initialize Variables
q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
qt=TrigPoly(q,s,c);
qd=msspoly('qd',nq);
qdd=msspoly('qdd',nq);
u=msspoly('u',nu);

% Set up known parameters
p=obj.getParamFrame.getPoly;
pobj = setParams(obj,p);

% Formulate equations of motion
[H,C,B] = manipulatorDynamics(pobj,qt,qd);
err = H*qdd + C - B*u;

[lp,M,Mb,lin_params,beta] = identifiableParameters(getmsspoly(err),p); % posynomial lumped params
% [lp, M, Mb] = linearParameters(getmsspoly(err),p); % monomial lumped params

nlp = length(lp);
lp_orig = double(subs(lp,p,p_orig));
lumped_params = msspoly('lp',nlp); 
% now err=M*lp+Mb and lperr=M*lumped_params+Mb;



%%   Step 2: Nonlinear least-squares estimation
% 
% Perform least squares on observable matrices: M_data*lp_est + Mb_data = 0

if (nargin>1)
  % populate A and b matrices from iddata
  % todo: make the processing of q,qd,qdd more robust
  Ts = get(data,'Ts');
  t_data = get(data,'SamplingInstants')';
  x_data = get(data,'OutputData')';
  q_data = x_data(1:nq,:);
  qd_data = x_data(nq+(1:nq),:);
  qdd_data = x_data(2*nq+(1:nq),:); 
  u_data = get(data,'InputData')';
end

s_data = sin(q_data);
c_data = cos(q_data);

ndata = length(t_data);
M_data = reshape(msubs(M(:),[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])',nq*ndata,nlp);
Mb_data = reshape(msubs(Mb,[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])',nq*ndata,1);


% Nonlinear least-squares solver
nonlinfun = @(x) M_data*(msubs(lp,p,x))+Mb_data;
[x, sqerr_est] = lsqnonlin(nonlinfun,p_orig,pmin,pmax);

% Computing the error rate
err_orig = M_data*lp_orig + Mb_data;
sqerr_orig = sum(err_orig'*err_orig);

phat = Point(getParamFrame(obj),x);


%%   Step 4: Print Results.
% 

if options.print_result == 1
    coords = getCoordinateNames(getParamFrame(obj));
    fprintf('\nParameter estimation results:\n\n');
    fprintf('  Param  \tOriginal\tEstimated\n');
    fprintf('  -----  \t--------\t---------\n');
    for i=1:length(coords)
      fprintf('%7s  \t%8.2f\t%8.2f\n',coords{i},p_orig(i),phat(i));
    end
elseif options.print_result == 2
    coords = getCoordinateNames(getParamFrame(obj));
    fprintf('\nParameter estimation results:\n\n');
    fprintf('  Param  \tEstimated\n');
    fprintf('  -----  \t---------\n');
    for i=1:length(coords)
      fprintf('%7s  \t%8.2f\n',coords{i},phat(i));
    end
end
%TODO: calculate estimated_delay
estimated_delay = 0;

end

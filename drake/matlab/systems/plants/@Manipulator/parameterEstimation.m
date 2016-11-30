function [phat,estimated_delay] = parameterEstimation(obj,data,varargin)
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
% 'noprint'     = no print, 
% 'printEst'    = only print estimated
% 'printAll'	= print estimated and original
% 
% @option model determines which model to use for estimation
% 'dynamic'     = use dynamic model - requires qdd
% 'energetic'   = use energetic model - doesn't require qdd
% 
% @option method determines which method to use for estimation
% 'nonlinprog'    = nonlinear least squares (LS) to solve problem
% 'linprog'       = linear LS on lumped params then nonlinear LS to recover
%                 original parameters
% 'simerr'      = minimize simulation error
% 'lsqnonlin'   = use MATLAB's built-in nonlinear least squares solver (debugging)

%% handle options
if (nargin>2 && isstruct(varargin{1})) options=varargin{1};
else options=struct(); end
if (~isfield(options,'print_result')) 
  options.print_result='noprint'; 
end
if (~isfield(options,'model')) 
  options.model='dynamic'; 
end
if (~isfield(options,'method')) 
  options.method='nonlinprog'; 
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

[H,C,B] = manipulatorDynamics(pobj,qt,qd);
if strcmp(options.model,'dynamic')
    % Formulate equations of motion
    err = H*qdd + C - B*u;
elseif strcmp(options.model,'energetic')
    dt=msspoly('dt',1);
    q1=msspoly('qo',nq);
    q2=msspoly('qf',nq);
    s1=msspoly('so',nq);
    s2=msspoly('sf',nq);
    c1=msspoly('co',nq);
    c2=msspoly('cf',nq);
    qt1=TrigPoly(q1,s1,c1);
    qt2=TrigPoly(q2,s2,c2);
    qd1=msspoly('qdto',nq);
    qd2=msspoly('qdtf',nq);
    % Formulate energy equations
    [T1,U1] = energy(pobj,[qt1;qd1]);
    [T2,U2] = energy(pobj,[qt2;qd2]);
    % Need to formulate energy dissipation from AcrobotPlant class
    dE = (B*u-[p(1);p(2)].*qd1)'*qd1*dt; % Under cursory testing, this works better
%     dE = (B*u-[p(1);p(2)].*qd1)'*(q2-q1);
    err = (T1+U1)-(T2+U2)+dE;
else
    error('Model not recognized')
end

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
  if strcmp(options.model,'dynamic')
    qdd_data = x_data(2*nq+(1:nq),:); 
  end
  u_data = get(data,'InputData')';
end

s_data = sin(q_data);
c_data = cos(q_data);
dt_data = diff(t_data);

% % Debugging the energy model
% E1 = getmsspoly(T1+U1);
% E = subs(E1,p,p_orig);
% E_data = msubs(E,[q1;s1;c1;qd1;u;dt],[q_data;s_data;c_data;qd_data;u_data;t_data]);
% diffE_data = diff(E_data);
% dE1 = subs(dE,p,p_orig);
% dE_data = msubs(dE1,[q1;qd1;q2;qd2;u;dt],[q_data(:,1:end-1);qd_data(:,1:end-1);...
%     q_data(:,2:end);qd_data(:,2:end);u_data(:,1:end-1);dt_data]);
% figure;plot(diffE_data,'-g'); hold on; plot(dE_data,'-r');
% title(['True \DeltaE vs. Estimated \DeltaE']);
% xlabel('Sample Time')
% ylabel({'\DeltaE Magnitude' '(Joules)/sample'})
% legend('True \DeltaE','Estimated \DeltaE');


if strcmp(options.model,'dynamic')
    ndata = length(t_data);
    variables = [q;s;c;qd;qdd;u];
    data_variables = [q_data;s_data;c_data;qd_data;qdd_data;u_data];
    M_data = reshape(msubs(M(:),variables,data_variables)',nq*ndata,nlp);
    Mb_data = reshape(msubs(Mb,variables,data_variables)',nq*ndata,1);
elseif strcmp(options.model,'energetic')
    variables = [q1;s1;c1;qd1;q2;s2;c2;qd2;u;dt];
    data_variables = [q_data(:,1:end-1);s_data(:,1:end-1);c_data(:,1:end-1);qd_data(:,1:end-1);...
        q_data(:,2:end);s_data(:,2:end);c_data(:,2:end);qd_data(:,2:end);u_data(:,1:end-1);dt_data];
    M_data = msubs(M(:),variables,data_variables)';
    Mb_data = msubs(Mb,variables,data_variables)';
end

if strcmp(options.method,'nonlinprog') || strcmp(options.method,'linprog')
    % Nonlinear least-squares solver
    prog = NonlinearProgram(np);
    prog=prog.addConstraint(BoundingBoxConstraint(pmin,pmax),1:np);
    
    if strcmp(options.method,'nonlinprog')
        % % Only nonlinear least squares
        % nonlinfun = @(x) nonlinerr(x,lp,p,M_data,Mb_data);
        % prog=prog.addCost(FunctionHandleObjective(np,nonlinfun),1:np);
    end
    if strcmp(options.method,'linprog')
        % Least squares -> Nonlinear least squares on lumped parameter solution
        lp_est = -pinv(full(M_data))*Mb_data;
        prog=prog.addQuadraticCost(eye(np),p_orig,1:np);
        lpconstraint_handle = @(x) lpconstraint_fun(x,lp,p);
        lpconstraint = FunctionHandleConstraint(lp_est,lp_est,nlp,lpconstraint_handle);
        prog=prog.addConstraint(lpconstraint,1:np);
    end
    [x,F,info] = prog.solve(p_orig);
    if(info ~= 1)
    	error('failed to solve the problem');
    end
elseif strcmp(options.method,'lsqnonlin')
    % Using MATLAB built-in nonlinear least squares solver
    % [x, sqerr_est] = lsqnonlin(nonlinfun,p_orig,pmin,pmax);
else
    error('Parameter estimation method not recognized')
end

phat = Point(getParamFrame(obj),x);

% % Computing the error rate
% err_orig = M_data*lp_orig + Mb_data;
% sqerr_orig = sum(err_orig'*err_orig);


%%   Step 4: Print Results.
% 

if strcmp(options.print_result,'printest')
    coords = getCoordinateNames(getParamFrame(obj));
    fprintf('\nParameter estimation results:\n\n');
    fprintf('  Param  \tEstimated\n');
    fprintf('  -----  \t---------\n');
    for i=1:length(coords)
      fprintf('%7s  \t%8.2f\n',coords{i},phat(i));
    end
elseif strcmp(options.print_result,'printall')
    coords = getCoordinateNames(getParamFrame(obj));
    fprintf('\nParameter estimation results:\n\n');
    fprintf('  Param  \tOriginal\tEstimated\n');
    fprintf('  -----  \t--------\t---------\n');
    for i=1:length(coords)
      fprintf('%7s  \t%8.2f\t%8.2f\n',coords{i},p_orig(i),phat(i));
    end
end
%TODO: calculate estimated_delay
estimated_delay = 0;

end

function [f,df] = lpconstraint_fun(x,lp,p)
    f = msubs(lp,p,x);
    dlpdp = diff(lp,p);
    % There must be a better way to msubs a matrix with spotless
    df = zeros(size(dlpdp,2));
    for i=1:size(dlpdp,2)
        df(:,i) = msubs(dlpdp(:,i),p,x);
    end
end

function [f,df] = nonlinerr(x,lp,p,M_data,Mb_data)
    sqrterr = M_data*(msubs(lp,p,x))+Mb_data;
    f = sqrterr'*sqrterr;
    dlpdp = diff(lp,p);
    % There must be a better way to msubs a matrix with spotless
    dlpdp_val = zeros(size(dlpdp,2));
    for i=1:size(dlpdp,2)
        dlpdp_val(:,i) = msubs(dlpdp(:,i),p,x);
    end
    df = 2*(M_data*(msubs(lp,p,x))+Mb_data)'*M_data*dlpdp_val;
end
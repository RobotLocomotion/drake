function [phat,simulation_error,estimated_delay,exitflag] = parameterEstimation(obj,data,varargin)
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
% 'simerr'      = minimize simulation error
% 
% @option method determines which method to use for estimation
% 'nonlinprog'    = nonlinear least squares (LS) to solve problem
% 'linprog'       = linear LS on lumped params then nonlinear LS to recover
%                 original parameters

% % DEBUGGING VARIABLES
% global numran 
% numran = 0;
% global TRAJFIG
% global PERRFIG;global PERR;global PERRMIN
% global PDIFFFIG;global PTRUE;global PDIFF


%% handle options
if (nargin>2 && isstruct(varargin{1})) options=varargin{1};
else options=struct(); end

% If parameters missing, use default values
if (~isfield(options,'print_result')) 
  options.print_result='noprint'; 
end
if (~isfield(options,'model')) 
  options.model='dynamic'; 
end
if (~isfield(options,'method')) 
  options.method='nonlinprog'; 
end
if (~isfield(options,'C'))
  options.C=eye(2*obj.num_positions);
end
if ~(strcmp(options.model,'dynamic') ||...
   strcmp(options.model,'energetic') ||...
   strcmp(options.model,'simerr'))
    error('Model not recognized')
end
if ~(strcmp(options.method,'nonlinprog') ||...
   strcmp(options.method,'linprog'))
    error('Method not recognized')
end

%% Initialize
if (getOutputFrame(obj)~=getStateFrame(obj))
  error('Only full-state feedback is implemented so far');
end
checkDependency('spotless');

nq = obj.num_positions;
nu = obj.num_u;
p_orig = double(getParams(obj));
np = length(p_orig);

[pmin,pmax] = getParamLimits(obj);
assert(all(pmin>=0));  % otherwise I'll have to subtract it out from the coordinates
%  not hard, just not implmented yet.

% Used often
isDynamic = strcmp(options.model,'dynamic');
isEnergetic = strcmp(options.model,'energetic');

%%   Step 1: Extract data
if (nargin>1)
  % populate A and b matrices from iddata
  % todo: make the processing of q,qd,qdd more robust
  Ts = get(data,'Ts');
  t_data = get(data,'SamplingInstants')';
  x_data = get(data,'OutputData')';
  q_data = x_data(1:nq,:);
  qd_data = x_data(nq+(1:nq),:);
  u_data = get(data,'InputData')';
  dt_data = diff(t_data);
  if isDynamic
    qdd_data = x_data(2*nq+(1:nq),:); 
  end
  s_data = sin(q_data);
  c_data = cos(q_data);
end
xobs = [q_data;qd_data];


%%   Step 2: Formulate error models
% Initialize Variables
q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
qt=TrigPoly(q,s,c);
qd=msspoly('qd',nq);
qdd=msspoly('qdd',nq);
u=msspoly('u',nu);
t=msspoly('t',1);

% Set up known parameters
p=obj.getParamFrame.getPoly;
pobj = setParams(obj,p);

[H,C,B] = manipulatorDynamics(pobj,qt,qd);

if isDynamic || isEnergetic
    if isDynamic
        % Formulate equation error from equations of motion
        err = H*qdd + C - B*u;
    elseif isEnergetic
        % If need memory efficiency, can cut down on # of msspoly
        % this is just a bit more readable
        % Initialize msspoly variables for expressing dE = E(t_2)-E(t_1)
        dt=msspoly('dt',1);
        
        q1=msspoly('qo',nq);
        s1=msspoly('so',nq);
        c1=msspoly('co',nq);
        qt1=TrigPoly(q1,s1,c1);
        qd1=msspoly('qdto',nq);
        
        q2=msspoly('qf',nq);
        s2=msspoly('sf',nq);
        c2=msspoly('cf',nq);
        qt2=TrigPoly(q2,s2,c2);
        qd2=msspoly('qdtf',nq);
        
        % Formulate energy equations
        [T1,U1] = energy(pobj,[qt1;qd1]);
        [T2,U2] = energy(pobj,[qt2;qd2]);
        
        % ACROBOT-SPECIFIC FORMULATION - MUST CHANGE
        % Need to formulate energy dissipation from AcrobotPlant class
        dE = (B*u-[p(1);p(2)].*qd1)'*qd1*dt;
        err = (T1+U1)-(T2+U2)+dE;
        
        % % DEBUGGING the energy model
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
    end
    % Isolate parameters from error equations
    [lp,M,Mb,lin_params,beta] = identifiableParameters(getmsspoly(err),p); % posynomial lumped params
    % [lp, M, Mb] = linearParameters(getmsspoly(err),p); % monomial lumped params
    nlp = length(lp);
    lp_orig = double(subs(lp,p,p_orig));
    lumped_params = msspoly('lp',nlp); 
    % now err=M*lp+Mb and lperr=M*lumped_params+Mb;
    
    if isDynamic
        ndata = length(t_data);
        variables = [q;s;c;qd;qdd;u];
        data_variables = [q_data;s_data;c_data;qd_data;qdd_data;u_data];
        M_data = reshape(msubs(M(:),variables,data_variables)',nq*ndata,nlp);
        Mb_data = reshape(msubs(Mb,variables,data_variables)',nq*ndata,1);
    elseif isEnergetic
        variables = [q1;s1;c1;qd1;q2;s2;c2;qd2;u;dt];
        data_variables = [q_data(:,1:end-1);s_data(:,1:end-1);c_data(:,1:end-1);qd_data(:,1:end-1);...
            q_data(:,2:end);s_data(:,2:end);c_data(:,2:end);qd_data(:,2:end);u_data(:,1:end-1);dt_data];
        M_data = msubs(M(:),variables,data_variables)';
        Mb_data = msubs(Mb,variables,data_variables)';
    end
elseif strcmp(options.model,'simerr')
    % Using MATLAB sym because need to represent rational functions
    % Perhaps a better way?
    [xdot,dxdot,ps,qs,qds,us,ts] = pobj.dynamicsSym([qt;qd],u,t);
    f = [xdot;dxdot];
end


%%   Step 3: Nonlinear least-squares estimation
% Nonlinear least-squares solver
prog = NonlinearProgram(np);
prog=prog.addConstraint(BoundingBoxConstraint(pmin,pmax),1:np);
nx = length(xobs(:,1));

if strcmp(options.model,'simerr')    
    dfdx = matlabFunction(jacobian(f*ts,[qs;qds]),'Vars',[ps;qs;qds;us;ts]);
    dfdp = matlabFunction(jacobian(f*ts,ps),'Vars',[ps;qs;qds;us;ts]);
    nonlinfun = @(px) simerr(obj,xobs,u_data,px,t_data,options.C,dfdx,dfdp);
    prog=prog.addCost(FunctionHandleObjective(np,nonlinfun),1:np);
    prog=prog.setSolverOptions('snopt','IterationsLimit',50000);
    prog=prog.setSolverOptions('snopt','MajorIterationsLimit',50000);
    prog=prog.setSolverOptions('snopt','MajorOptimalityTolerance',1.0e-5);
%     % FOR DEBUGGING SIMERR
%     if exist('TRAJFIG','var'),TRAJFIG = figure;end
%     if exist('PERRFIG','var'),PERRFIG = figure; PERR = 10^30*ones(1,100); PERRMIN = min(PERR);end
%     if exist('PDIFFFIG','var'),PDIFFFIG = figure; PDIFF = PTRUE-p_orig;end
elseif strcmp(options.method,'nonlinprog')
    % Only nonlinear least squares
    nonlinfun = @(x) nonlinerr(x,lp,p,M_data,Mb_data);
    prog=prog.addCost(FunctionHandleObjective(np,nonlinfun),1:np);
elseif strcmp(options.method,'linprog')
    % Least squares -> Nonlinear least squares on lumped parameter solution
    lp_est = -pinv(full(M_data))*Mb_data;
    prog=prog.addQuadraticCost(eye(np),p_orig,1:np);
    lpconstraint_handle = @(x) lpconstraint_fun(x,lp,p);
    lpconstraint = FunctionHandleConstraint(lp_est,lp_est,nlp,lpconstraint_handle);
    prog=prog.addConstraint(lpconstraint,1:np);
end
[x,simulation_error,exitflag] = prog.solve(p_orig);
% if(exitflag ~= 1)
%     % Use MATLAB built-in nonlinear least squares solver if prog fails
%     % lsqnonlin doesn't work for either scenarios
%     if strcmp(options.method,'linprog'); error('linprog failed'); end
%     if strcmp(options.method,'simerr'); error('simerr failed'); end %
%     
%     warning('NonlinearProgram failed - trying lsqnonlin')
%     [x,sqerr_est,~,flag] = lsqnonlin(nonlinfun,p_orig,pmin,pmax);
%     if(flag ~= 1), error('lsqnonlin failed');end
% end

phat = Point(getParamFrame(obj),x);

%% Computing the simulation error
% err_orig = M_data*lp_orig + Mb_data;
% sqerr_orig = sum(err_orig'*err_orig);
if strcmp(options.method,'nonlinprog') || strcmp(options.method,'linprog')
    simulation_error = simerr(obj,xobs,u_data,x,t_data,options.C);
end

%%   Step 4: Print Results.
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

function [F,dF] = simerr(obj,xobs,utraj,p,t,C,varargin)
%     global numran; numran = numran+1;
%     global TRAJFIG; 
%     global PERRFIG; global PERR; global PERRMIN; 
%     global PDIFFFIG;global PTRUE;global PDIFF;
    
    newobj = obj.setParams(p);
    x0 = xobs(:,1);
    error = false;
    try
        xtraj = computeTraj(newobj,x0,utraj,t);
        xdiff = xtraj - xobs;
        F = sum(sum(xdiff.*(C*xdiff),1),2);
    catch err
        % The system parameters are unstable, output infinite error
        if strcmp(err.identifier,'MATLAB:svd:matrixWithNaNInf')
            error = true;
            F = Inf;
        else rethrow(err);end
    end
    if (nargout>1)
        if ~error
            if ~(nargin>7); error('Requires input dfdx and dfdp'); end
            dfdx = varargin{1};
            dfdp = varargin{2};
            dt = diff(t);
            N = length(dt);
            I = eye(length(x0));
            args = num2cell([p;x0;utraj(:,1);dt(1)]');
            dx = cell(1,N);dx{1} = dfdp(args{:});
            dF = xdiff(:,2)'*C*dx{1};
            for i=2:(N)
                args = num2cell([p;xtraj(:,i);utraj(:,i);dt(i)]');
                dx{i} = (I+dfdx(args{:}))*dx{i-1}+dfdp(args{:});
                dF = dF + xdiff(:,i+1)'*C*dx{i};
            end
            dF = 2*dF;
        else
            dF = -Inf(1,length(p));
        end
% %       % FOR DEBUGGING SIMERR
%         if error
%             xtraj = zeros(size(xobs));
%         end
%         if mod(numran,100) == 0
%             fprintf('%i ',numran);
%         end
%         if exist('TRAJFIG','var') 
%             figure(TRAJFIG);
%             hold off;plot(xobs(1,:),xobs(2,:)); 
%             ax = gca; xlim = ax.XLim; ylim = ax.YLim;
%             hold on; plot(xtraj(1,:),xtraj(2,:)); 
%             set(gca,'XLim',xlim,'YLim',ylim); drawnow; 
%         end
%         if exist('PERRFIG','var') 
%             figure(PERRFIG);
%             PERRMIN = min([PERRMIN F]); 
%             PERR = [PERR PERRMIN];
%             hold off;plot((numran+1-100):numran,PERR((end+1-100):end)');drawnow;
%         end
%         if exist('PDIFFFIG','var') 
%             figure(PDIFFFIG); 
%             PDIFF = [PDIFF PTRUE-p];
%             try
%             hold off;plot(0:numran,PDIFF');drawnow;
%         end
    end
end

function xtraj = computeTraj(obj,x0,utraj,t)
    nx = length(x0);
    dt = diff(t);
    N = length(dt);
    xtraj = [x0,zeros(nx,N)];
    for i=1:N
        f = dynamics(obj,t(i),xtraj(:,i),utraj(:,i));
        xtraj(:,i+1) = xtraj(:,i)+f*dt(i);
    end
end
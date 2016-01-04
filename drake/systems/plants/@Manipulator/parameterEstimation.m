function [phat,simulation_error,estimated_delay] = parameterEstimation(obj,data,varargin)
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

% global numran 
% numran = 0;
% global pp
% global ppp

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
p_orig = double(getParams(obj));  % probably only for testing
np = length(p_orig);

% Should be passed into as function param
O = diag([1 1 1 1]);

[pmin,pmax] = getParamLimits(obj);
pmin = pmin + 1e-3;
assert(all(pmin>0));  % otherwise I'll have to subtract it out from the coordinates
%  not hard, just not implmented yet.


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
  if strcmp(options.model,'dynamic')
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
isDynamic = strcmp(options.model,'dynamic');
isEnergetic = strcmp(options.model,'energetic');
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
        
        % ACROBOT-SPECIFIC FORMULATION - MUST CHANGE
        % Need to formulate energy dissipation from AcrobotPlant class
        dE = (B*u-[p(1);p(2)].*qd1)'*qd1*dt; % Under cursory testing, this works better
        % dE = (B*u-[p(1);p(2)].*qd1)'*(q2-q1); % than this
        err = (T1+U1)-(T2+U2)+dE;
        
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

if strcmp(options.model,'simerr')    
    dfdx = matlabFunction(jacobian(f*ts,[qs;qds]),'Vars',[ps;qs;qds;us;ts]);
    dfdp = matlabFunction(jacobian(f*ts,ps),'Vars',[ps;qs;qds;us;ts]);
    nonlinfun = @(px) simerr(obj,xobs,u_data,px,t_data,O,dfdx,dfdp);
    prog=prog.addCost(FunctionHandleObjective(np,nonlinfun),1:np);
%     % FOR DEBUGGING SIMERR
%     pp = ppp-p_orig;
%     figure; hold off;
%     figure; plot(xobs(1,:),xobs(2,:));drawnow;hold on;
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
[x,Fsym,info] = prog.solve(p_orig);
if(info ~= 1)
    % Use MATLAB built-in nonlinear least squares solver if prog fails
    if strcmp(options.method,'linprog'); error('linprog failed'); end
    warning('NonlinearProgram failed - trying lsqnonlin')
    [x,sqerr_est,~,flag] = lsqnonlin(nonlinfun,p_orig,pmin,pmax);
    if(flag ~= 1)
        error('lsqnonlin failed');
    end
end

phat = Point(getParamFrame(obj),x);

%% Computing the simulation error
% err_orig = M_data*lp_orig + Mb_data;
% sqerr_orig = sum(err_orig'*err_orig);
if strcmp(options.model,'simerr')    
    simulation_error = Fsym;
elseif strcmp(options.method,'nonlinprog') || strcmp(options.method,'linprog')
    simulation_error = simerr(obj,xobs,u_data,x,t_data,O);
end

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

function [F,dF] = simerr(obj,xobs,utraj,p,t,O,varargin)
%     global numran; numran = numran+1;
%     global pp;global ppp
    newobj = obj.setParams(p);
    x0 = xobs(:,1);
    xtraj = computeTraj(newobj,x0,utraj,t);
    xdiff = xtraj - xobs;
%     % ACROBOT-SPECIFIC - MUST CHANGE
%     xdiff(1,:) = mod(xdiff(1,:)+pi,2*pi)-pi;
%     xdiff(2,:) = mod(xdiff(2,:)+pi,2*pi)-pi;

    F = sum(sum(xdiff.*(O*xdiff),1),2);
    if (nargout>1)
        if ~(nargin>7); error('Requires input dfdx and dfdp'); end
        dfdx = varargin{1};
        dfdp = varargin{2};
        dt = diff(t);
        N = length(dt);
        I = eye(length(x0));
        args = num2cell([p;x0;utraj(:,1);dt(1)]');
        dx = cell(1,N);dx{1} = dfdp(args{:});
        dF = xdiff(:,2)'*O*dx{1};
        for i=2:(N)
            args = num2cell([p;xtraj(:,i);utraj(:,i);dt(i)]');
            dx{i} = (I+dfdx(args{:}))*dx{i-1}+dfdp(args{:});
            dF = dF + xdiff(:,i+1)'*O*dx{i};
        end
        dF = 2*dF;
%         % FOR DEBUGGING SIMERR
%         if mod(numran,1) == 0
%             disp(numran);disp(F);disp(dF);disp(p')
%             disp(ppp'-p');pp = [pp ppp-p];plot(0:numran,pp')
%             plot(xtraj(1,:),xtraj(2,:)); drawnow;
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


%% Code Graveyard
%     xdata = [q_data;qd_data];
%     x0 = xdata(:,1);
%     ndata = length(t_data);
%     nx = length(x0);
%     dt = diff(t_data);
%     dx = matlabFunction(f*ts,'Vars',[ps;qs;qds;us;ts]);
%     
%     xsym = cell(1,ndata);
%     xsym{1} = x0;
%     
%     tic
%     Fsym = 0;
%     for i=2:ndata
% %         if mod(i,5)==0
%             disp(i);
% %         end
%         args = num2cell([ps;xsym{i-1};u_data(:,i-1);dt(i-1)]');
%         dxi = dx(args{:});
%         xsym{i} = simplify(xsym{i-1}+dxi);
%         
%         xdiffi = xsym{i}-xdata(:,i); 
%         Fsym = Fsym + xdiffi'*O*xdiffi;
%     end
%     F = matlabFunction(Fsym);
%     dF = matlabFunction(jacobian(Fsym,ps),'Vars',[ps;qs;qds;us;ts]);
%     toc

% function [F,dF] = simerr(f,xsym,usym,psym,xobs,utraj,p,dt,O)
%     global numran
%     numran = numran+1
%     tic
%     x0 = xobs(:,1);
%     fp = subs(f,psym,p);
%     xtraj = computeTraj(fp,xsym,usym,x0,utraj,dt);
%     xdiff = xtraj - xobs;
%     F = sum(sum(xdiff.*(O*xdiff),1),2)
%     if (nargout>1)
%         N = length(dt);
%         u0 = utraj(:,1);
%         dF = 0;
%         dfdx = @(xi,ui,dti) double(subs(jacobian(fp,xsym),[xsym;usym],[xi;ui]))*dti;
%         dx = cell(1,N);
%         dx{1} = double(subs(jacobian(subs(f,[xsym;usym],[x0;u0]),psym),psym,p))*dt(1);
%         I = eye(length(x0));
%         for i=2:(N)
% %             disp(i);
%             dx{i} = (I+dfdx(xobs(:,i),utraj(:,i),dt(i)))*dx{i-1};
%             dF = dF + xdiff(:,i+1)'*O*dx{i};
%         end
%         dF = 2*dF;
%     end
%     toc
% end

% function xtraj = computeTraj(f,xsym,usym,x0,utraj,dt)
%     nx = length(x0);
%     N = length(dt);
%     xtraj = [x0,zeros(nx,N)];
%     for i=1:N
%         xi = xtraj(:,i);
%         xtraj(:,i+1) = xi+double(subs(f,[xsym;usym],[xi;utraj(:,i)])*dt(i));
%     end
% end

% H=[(1)+ps(3)^2+ps(4)^2+(2)*ps(4)*cs(2)+ps(5)+ps(6), ps(4)^2+ps(4)*cs(2)+ps(6); ps(4)^2+ps(4)*cs(2)+ps(6), ps(4)^2+ps(6)];
% 
% buc=[(-9.81)*ss(1)-qds(1)*ps(1)+(-9.81)*ps(3)*ss(1)+(-9.81)*ps(4)*cs(2)*ss(1)+(-9.81)*ps(4)*ss(2)*cs(1)+ps(4)*qds(2)^2*ss(2)+(2)*ps(4)*qds(2)*ss(2)*qds(1),...
%     us(1)-qds(2)*ps(2)+(-9.81)*ps(4)*cs(2)*ss(1)+(-9.81)*ps(4)*ss(2)*cs(1)-ps(4)*ss(2)*qds(1)^2];
% 
% inv(H)*buc
% [(1)+p3^2+p4^2+(2)*p4*c2+p5+p6, p4^2+p4*c2+p6; p4^2+p4*c2+p6, p4^2+p6]
% [(-9.81)*s1-qd1*p1+(-9.81)*p3*s1+(-9.81)*p4*c2*s1+(-9.81)*p4*s2*c1+p4*qd2^2*s2+(2)*p4*qd2*s2*qd1,...
%     u1-qd2*p2+(-9.81)*p4*c2*s1+(-9.81)*p4*s2*c1-p4*s2*qd1^2]

% tn = 10;
% f = dynamics(pobj,t_data(:,tn),[q_data(:,tn);qd_data(:,tn)],u_data(:,tn))
% f = dynamics(pobj,t,[qt;qd],u);


% 
%         u0 = utraj(:,1);
%         dt0 = dt_data(1);
%         f0 = subs(f,[xsym;usym],[x0;u0]);
%         x0_f0 = x0+f0*dt0;
%         const1 = N*(x0_f0)'*O*(f0*dt0);
%         dconst1 = gradient(const1,psym);
%         const2 = -2*sum(xobs(:,2:end),2)'*O*(f0*dt0);
%         dconst2 = gradient(const2,psym);
%         dconst = dconst1+dconst2;
%         dF = double(subs(dconst,psym,p));
%       
% 
%         
%         x0_f0_d = double(subs(x0_f0,psym,p));
%         x0_f0_mat = repmat(x0_f0_d,[1,size(xtraj,2)]);
%         A = x0_f0_mat-xobs;
%         B = xtraj-x0_f0_mat;
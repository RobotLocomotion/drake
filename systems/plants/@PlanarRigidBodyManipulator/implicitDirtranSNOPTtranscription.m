function [w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,userfun,wrapupfun,iname,oname] = implicitDirtranSNOPTtranscription(sys,costFun,finalCostFun,x0,utraj0,con,options)
%  This function should not be called directly.  Use the
%  trajectoryOptimization interface. 

if (~isfield(options,'xtape0')) options.xtape0='free'; end
% nL = sys.num_bilateral_constraints - sys.getNumContacts;
nL = sys.getNumJointLimitConstraints;
nC = sys.num_contacts;
nU = sys.getNumInputs();
nX = sys.getNumContStates();
nXd = sys.getNumDiscStates();
nClutch = 0;

if (nXd>0)
  error('not implemented yet (but should be very straightforward)');
end

ts = sys.getSampleTime();
if (size(ts,2)>1 || any(ts~=0))
  warning('found non-continuous sample times.  i''ll proceed assuming everything is continuous'); 
%  error('not implemented yet (but should be very straightforward)');
end

if isfield(options,'time')
  t = options.time;
else
  t = utraj0.getBreaks(); 
end

if ~isfield(options,'trimgrad')
  options.trimgrad = 1;
end

t=t(:);
nT = length(t);
tscale = ones(nT-1,1);  % stretch time by this amount
u = utraj0.eval(t);


lambda = zeros(nL,length(t));
cLambda = zeros(nC*2,length(t));
clutch = zeros(nClutch,length(t));

if (strcmp(options.xtape0,'simulate'))
  drivensys = cascade(utraj0,sys);
  xtraj = simulate(drivensys,[t(1),t(end)],x0);
  x = xtraj.eval(t);
elseif  (strcmp(options.xtape0,'tape'))
  if ~isfield(options,'xtrajOffset')
    options.xtrajOffset = 0;
  end
  x = options.traj0.x.eval(t + options.xtrajOffset);  
  if (nClutch + nC + nL > 0)
    fullLambda = options.traj0.lambda.eval(t + options.xtrajOffset);
    lambda = fullLambda(1:nL,:);
    cLambda = fullLambda(nL+1:nL+2*nC,:);
    clutch = fullLambda(nL+2*nC+(1:nClutch),:);
  end
else % options.xtape0='rand' or 'free'
  x = repmat(x0,1,length(t));%+.1*randn(length(x0),length(t));   x(:,1) = x0;
end
  
nJointLimitConst = sys.getNumJointLimitConstraints;

%% Set up snopt inputs
w0 = [tscale; x(:); u(:); lambda(:); cLambda(:); clutch(:)];
whigh = [ inf(size(tscale)); repmat(inf,prod(size(x)),1); repmat(inf,prod(size(u)),1); repmat(inf,prod(size(lambda)),1); repmat(inf,prod(size(cLambda)),1); repmat(1,prod(size(clutch)),1)];
wlow = [ zeros(size(tscale)); repmat(-inf,prod(size(x)),1); repmat(-inf,prod(size(u)),1); repmat([zeros(nJointLimitConst,1);-inf(nL-nJointLimitConst,1)],nT,1); repmat([-inf;0],nC*length(t),1); repmat(0,prod(size(clutch)),1)];
A=[];
iAfun=[];
jAvar=[];

% handle constraints
  function conwarn(f1,f2)
    if (iscell(f1)) f1 = f1{1}; end
    if (nargin>1)
      if (iscell(f2)) f2 = f2{1}; end
      warning([f1,'.',f2,' constraint is not supported by dirtran (at least not yet)']);
    else
      warning([f1,' constraint is not supported by dirtran (at least not yet)']);
    end      
  end

for f1 = fieldnames(con)'
  switch(f1{1})
    case 'x'
      xind=nT-1+(1:prod(size(x)));
      for f2 = fieldnames(con.x)'
        switch(f2{1})
          case 'lb'
            c = repmat(con.x.lb,nT,1);
            wlow(xind) = max(wlow(xind),c);
          case 'ub'
            c = repmat(con.x.ub,nT,1);
            whigh(xind) = min(whigh(xind),c);
          case 'c'
            % intentionally pass through... these get handled in the userfun
          otherwise
            conwarn(f1,f2);
        end
      end
    case 'clambdai'
      clind = nT-1+ nT*(nX + nU + nL) + 1 : nT-1+ nT*(nX + nU + nL + 2*nC);
      
      for f2 = fieldnames(con.clambdai)'
        switch(f2{1})
          case 'ub'
            indices = con.clambdai.ub.i;
            windices = kron((indices(:)-1)*2*nC,ones(2*nC,1))+repmat((1:2*nC)',length(indices),1);
            whigh(clind(windices)) = min(whigh(clind(windices)),con.clambdai.ub.clambdai(:));
          case 'lb'
            indices = con.clambdai.lb.i;
            windices = kron((indices(:)-1)*2*nC,ones(2*nC,1))+repmat((1:2*nC)',length(indices),1);
            wlow(clind(windices)) = max(wlow(clind(windices)),con.clambdai.lb.clambdai(:));
          otherwise
            conwarn(f1,f2);
        end
      end
    case 'clutchi'
      clutchind = nT-1+ nT*(nX + nU + nL + 2*nC) + 1 : nT-1+ nT*(nX + nU + nL + 2*nC + nClutch);
      
      for f2 = fieldnames(con.clutchi)'
        switch(f2{1})
          case 'ub'
            indices = con.clutchi.ub.i;
            windices = kron((indices(:)-1)*nClutch,ones(nClutch,1))+repmat((1:nClutch)',length(indices),1);
            whigh(clutchind(windices)) = min(whigh(clutchind(windices)),con.clutchi.ub.clutchi(:));
          otherwise
            conwarn(f1,f2);
        end
      end
    case 'x0'
      for f2 = fieldnames(con.x0)'
        switch(f2{1})
          case 'lb'
            wlow(nT-1+ (1:nX)) = max(wlow(nT-1+ (1:nX)),con.x0.lb);
          case 'ub'
            whigh(nT-1+ (1:nX)) = min(whigh(nT-1+ (1:nX)),con.x0.ub);
          case {'c','ceq'}
            % intentionally pass through.. these get handled in the userfun
          otherwise 
            conwarn(f1,f2);
        end
      end
    case 'xf'
      xfind = nT-1+ nX*(nT-1) + (1:nX);
      for f2 = fieldnames(con.xf)'
        switch(f2{1})
          case 'lb'
            wlow(xfind) = max(wlow(xfind),con.xf.lb);
          case 'ub'
            whigh(xfind) = min(whigh(xfind),con.xf.ub);
          case {'c','ceq'}
            % intentionally pass through... these get handled in the userfun
          otherwise 
            conwarn(f1,f2);
        end
      end
      if strcmp(options.xtape0,'free') && all(wlow(xfind)==whigh(xfind))  % final value constraint
        for i=1:nX
          x(i,:) = linspace(x0(i),wlow(xfind(i)),nT);
        end
        w0(nT-1+(1:(nT*nX))) = x(:);
      end
    case 'u'
      for f2 = fieldnames(con.u)'
        uind = nT-1+ nX*nT + 1 : nT-1+ nX*nT + nU*nT;
        switch(f2{1})
          case 'lb'
            wlow(uind) = max(wlow(uind),repmat(con.u.lb,nT,1));
          case 'ub'
            whigh(uind) = min(whigh(uind),repmat(con.u.ub,nT,1));
          otherwise 
            conwarn(f1,f2);
        end
      end
    case 'u0'
      for f2 = fieldnames(con.u0)'
        uind = nT-1+ nX*nT + (1 : nU);
        switch(f2{1})
          case 'lb'
            wlow(uind) = max(wlow(uind),con.u0.lb);
          case 'ub'
            whigh(uind) = min(whigh(uind),con.u0.ub);
          otherwise 
            conwarn(f1,f2);
        end
      end
    case 'uf'
      for f2 = fieldnames(con.uf)'
        uind = nT-1+ nX*nT + nU*(nT-1) + (1 : nU);
        switch(f2{1})
          case 'lb'
            wlow(uind) = max(wlow(uind),con.uf.lb);
          case 'ub'
            whigh(uind) = min(whigh(uind),con.uf.ub);
          otherwise 
            conwarn(f1,f2);
        end
      end
      
    case 'T'
    case 'periodic'
    case 'linperiodic'
    case 'linperiodic_min'
    case 'linperiodic_max'
    case 'fixtime'
    case 'noflight'
    case 'betamult'
    case 'alphamult'
            % intentionally pass through.  this is implemented in userfun_ind  
    otherwise
      conwarn(f1);
  end
end

if (options.grad_test)
  % print out the debugging key:
  iname = {'tscale'};
  for i=1:nT, for j=1:nX, iname= {iname{:},['x_',num2str(j),'(',num2str(i),')']}; end, end
  for i=1:nT, for j=1:nU, iname= {iname{:},['u_',num2str(j),'(',num2str(i),')']}; end, end
  
  [nf,A,iAfun,jAvar,iGfun,jGvar,Fhigh,Flow,oname] = userfun_grad_ind(sys,nT,nX,nU,nL,nC,nClutch,con,x,t,options);
else
  [nf,A,iAfun,jAvar,iGfun,jGvar,Fhigh,Flow] = userfun_grad_ind(sys,length(t),nX,nU,nL,nC,nClutch,con,x,t,options);
  iname={};
  oname={};
end

if options.trimgrad
  if ~isfield(options,'grad_I') || ~isfield(options,'grad_skip')
    options.grad_I = 1:length(iGfun);
    options.grad_skip = [];
    userfun = @(w) dirtran_userfun(sys,w,costFun,finalCostFun,t,nX,nU,nL,nC,nClutch,con,options);
    % Lets try to remove some of these indices
    GG = zeros(length(iGfun),100);
    for i=1:100,
      [~,GG(:,i)] = userfun(1e6*randn(size(w0)));
    end
    grad_I = find(sum(GG.^2,2) ~= 0);
    grad_skip = find(sum(GG.^2,2) == 0);
    
    options.grad_I = grad_I;
    options.grad_skip = grad_skip;
    
    userfun = @(w) dirtran_userfun(sys,w,costFun,finalCostFun,t,nX,nU,nL,nC,nClutch,con,options);
    
    fprintf('Checked and removed %d out of %d total sparse gradient elements\n',length(grad_skip),length(jGvar));
  else
    fprintf('Removed %d out of %d total sparse gradient elements\n',length(options.grad_skip),length(jGvar));  
  end
  userfun = @(w) dirtran_userfun(sys,w,costFun,finalCostFun,t,nX,nU,nL,nC,nClutch,con,options);
  jGvar = jGvar(options.grad_I);
  iGfun = iGfun(options.grad_I);
else
  options.grad_I = 1:length(iGfun);
  options.grad_skip = [];
  userfun = @(w) dirtran_userfun(sys,w,costFun,finalCostFun,t,nX,nU,nL,nC,nClutch,con,options);
end

wrapupfun = @(w) dirtran_wrapup(sys,w,t,nX,nU,nL,nC,nClutch,options);
end

function [f,G] = dirtran_userfun(sys,w,costFun,finalCostFun,tOrig,nX,nU,nL,nC,nClutch,con,options)
  nT = length(tOrig);
  t = [tOrig(1);zeros(nT-1,1)];
  for i=1:nT-1
      t(i+1) = t(i) + (tOrig(i+1) - tOrig(i))*w(i);
  end
  
%   dtdw1 = tOrig;
  dt = w(1:nT-1).*diff(tOrig);
  ddtdw1 = diff(tOrig);
  
  x = reshape(w(nT-1+[1:(nT*nX)]),nX,nT);
  u = reshape(w((nT+nT*nX):(nT-1+nT*nX+nT*nU)),nU,nT);
  lambda = reshape(w((nT+nT*(nX+nU)):(nT-1+nT*(nX+nU+nL))),nL,nT);
  cLambda = reshape(w((nT+nT*(nX+nU+nL)):(nT-1+nT*(nX+nU+nL+2*nC))),2*nC,nT);
  clutch = reshape(w((nT+nT*(nX+nU+nL+2*nC)):(nT-1+nT*(nX+nU+nL+2*nC+nClutch))),nClutch,nT);

%   tcol = t(1:end-1)+dt/2;  dtcoldw1 = dtdw1(1:end-1)+ddtdw1/2;
%   ucol = .5*(u(:,1:end-1)+u(:,2:end));  
%   
%  figure(1); clf; plot(x(1,:),x(2,:)); drawnow;

  num_q = sys.num_positions;
  %% todo: vectorize this when possible

  % preallocate vars
  g = zeros(size(dt));            %cost over time
  dg = zeros(1,1+nX+nU,nT-1);     %cost gradient g(1,i,j) = dg(j)/dx(i)
  d = zeros(nX+2*nL+5*nC,nT-1);             %time derivative dy/dt at collocation point
  dd = zeros(nX+2*nL+5*nC,1+2*nX+2*nU+2*nL+4*nC+2*nClutch,nT-1);%gradient of d at collocation point
  
  bxc = isfield(con,'x') && isfield(con.x,'c');  % eval boolean once (not repititively inside the loop)
  fxc = [];  Gxc=[];  

  % iterate through time
  for i=1:(nT-1)  % compute dynamics and cost 
    d_i = zeros(nX+2*nL+5*nC,1);
    dd_i = zeros(nX+2*nL+5*nC,1+2*nX+2*nU+2*nL+4*nC+2*nClutch,1);
%     dg_i = zeros(1+nX+nU,1);
    
    %Scale gradient dg(i)/dtscale by the original time (why?)    
%     dg(1,1,i)=dg(1,1,i)*dtdw1(i);  % d/d[tscale; x(:,i); u(:,i)]
    
%     dg(1,:,i) = [g(i)*ddtdw1(i),zeros(1,nX+nU)]+dt(i)*dg(1,:,i);  g(i) = g(i)*dt(i);
    
    cLMult = 100;
    cMult = 1;
    
%     [H,dH] = sys.getH(x(1:num_q,i+1));
    % Backwards euler integration
    k = 50;
        clutch_var = clutch(:,i+1);
    %     clutch_var = 1./(1+exp(-2*k*(clutch(:,i+1)-.5)));  %use step fun approximation
    
    % hack for fastrunner only
%     if nClutch == 2
%       clutch_var(1) = (1-exp(-100*(cLambda(2)+cLambda(4)+cLambda(6))));
%       clutch_var(2) = (1-exp(-100*(cLambda(8)+cLambda(10)+cLambda(12))));
%     end

    [Hqddot, dHqddot, phi, psi, dPhi, dPsi, H, dH] = sys.implicitDynamics(t(i+1),x(:,i+1),u(:,i+1),lambda(:,i+1)*cMult,cLambda(:,i+1)*cLMult);
    
    dHqddot(:,(2+nX+nU):(1+nX+nU+nL)) = dHqddot(:,(2+nX+nU):(1+nX+nU+nL))*cMult;
    dHqddot(:,(2+nX+nU+nL):(1+nX+nU+nL+2*nC)) = dHqddot(:,(2+nX+nU+nL):(1+nX+nU+nL+2*nC))*cLMult;
    
    
%     dHqddot(:,(2+nX+nU+nL+2*nC):(1+nX+nU+nL+2*nC+nClutch)) =  dHqddot(:,(2+nX+nU+nL+2*nC):(1+nX+nU+nL+2*nC+nClutch))*diag((2*k*exp(-2*k*(clutch(:,i+1)-.5)).*clutch_var.^2));
%     dHqddot(:,(2+nX+nU+nL+2*nC):(1+nX+nU+nL+2*nC+nClutch)) =  dHqddot(:,(2+nX+nU+nL+2*nC):(1+nX+nU+nL+2*nC+nClutch))*0;
    dqdot = x(num_q+1:end,i+1) - x(num_q+1:end,i);
    %
    d_i(1:num_q) = x(1:num_q,i) - x(1:num_q,i+1) + dt(i)*x(num_q+1:end,i+1);
    dd_i(1:num_q,:) = [ddtdw1(i)*x(num_q+1:end,i+1) eye(num_q) ...
      zeros(num_q,num_q+ nU+nL+2*nC+nClutch) -eye(num_q) dt(i)*eye(num_q) ...
      zeros(num_q,nU+nL+2*nC+nClutch)];
    
    
%    d_i(1:num_q) = (x(1:num_q,i) - x(1:num_q,i+1))/dt(i) + x(num_q+1:end,i+1);
%    dd_i(1:num_q,:) = [-ddtdw1(i)/(dt(i)^2)*(x(1:num_q,i) - x(1:num_q,i+1)) eye(num_q)/dt(i)...
%      zeros(num_q,num_q+ nU+nL+2*nC+nClutch) -eye(num_q)/dt(i) eye(num_q) ...
%      zeros(num_q,nU+nL+2*nC+nClutch)];
    
    d_i(num_q+1:nX) = H*dqdot - dt(i)*Hqddot;
    dd_i(num_q+1:num_q*2,:) = [-ddtdw1(i)*Hqddot zeros(num_q) -H zeros(num_q,nU+nL+2*nC+nClutch)...
      matGradMult(dH(:,1:num_q),dqdot) H zeros(num_q,nU + nL+2*nC+nClutch)] - [zeros(num_q,1+nX+nU+nL+2*nC+nClutch) dt(i)*dHqddot(:,2:end)];
    
    %trying a 1e3 scaling on dynamic constraint
%     d_i(num_q+1:nX) = d_i(num_q+1:nX)*1e3;
%     dd_i(num_q+1:num_q*2,:) = dd_i(num_q+1:num_q*2,:)*1e3;
    
%    d_i(num_q+1:nX) = 1/dt(i)*H*dqdot - Hqddot;
%    dd_i(num_q+1:num_q*2,:) = [-ddtdw1(i)/(dt(i)^2)*H*dqdot zeros(num_q) -H/dt(i) zeros(num_q,nU+nL+2*nC+nClutch)...
%      1/dt(i)*matGradMult(dH,dqdot) H/dt(i) zeros(num_q,nU + nL+2*nC+nClutch)] - [zeros(num_q,1+nX+nU+nL+2*nC+nClutch) dHqddot(:,2:end)];

    %position constraints
    %     if nL > 0
    %       [phi,J] = geval(@sys.positionConstraints,x(1:num_q,i));
    %       d(nX+1:nX+nL,i) = phi;
    %       dd(nX+1:nX+nL,:,i) = [zeros(nL,1) J zeros(nL,num_q+nU*2+nL*2+nX)];
    %     end
        
    %contact constraints, currently assuming the ground normal is up and
    %the ground is flat
    if nC >0
%       [phi, psi, dPhi, dPsi] = sys.contactPositionsAndVelocities(x(1:num_q,i+1),x(num_q+1:end,i+1));
      for j=1:nC,
        contactMult = 1;%1e-4;
        %2 equality constraints first        
        % z-force(i) * z-height(i+1) = 0  (force only non-zero when height is zero)
        d_i(nX+2*nL+1+(j-1)*5) = cLambda(2*j,i+1) * phi(2*j)*contactMult;
        dd_i(nX+2*nL+1+(j-1)*5,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j) = phi(2*j)*contactMult;
        dd_i(nX+2*nL+1+(j-1)*5,(1:num_q) + 1+nX+nU+nL+2*nC+nClutch) = cLambda(2*j,i+1) * dPhi(2*j,:)*contactMult;
%         
        % use (a - s(a-b,epsilon))^2 instead of complementarity where 
        % s(x,epsilon)=epsilon*log(1+exp(x/epsilon))
%         epsilon= .001;
%         exponent = (cLambda(2*j,i+1)-phi(2*j))/epsilon;
%         if exponent < 100
%           d_i(nX+2*nL+1+(j-1)*4) = (cLambda(2*j,i+1) - epsilon*log(1+exp((cLambda(2*j,i+1)-phi(2*j))/epsilon)))^2;
%           dd_i(nX+2*nL+1+(j-1)*4,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j) = (2*(cLambda(2*j,i+1) - epsilon*log(exp((cLambda(2*j,i+1) - phi(2*j))/epsilon) + 1)))/(exp((cLambda(2*j,i+1) - phi(2*j))/epsilon) + 1);
%           dd_i(nX+2*nL+1+(j-1)*4,(1:num_q) + 1+nX+nU+nL+2*nC+nClutch) = dPhi(2*j,:)*(2*exp((cLambda(2*j,i+1) - phi(2*j))/epsilon)*(cLambda(2*j,i+1) - epsilon*log(exp((cLambda(2*j,i+1) - phi(2*j))/epsilon) + 1)))/(exp((cLambda(2*j,i+1) - phi(2*j))/epsilon) + 1);
%         else
%            d_i(nX+2*nL+1+(j-1)*4) = phi(2*j)^2;
%            dd_i(nX+2*nL+1+(j-1)*4,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j) = 0;
%            dd_i(nX+2*nL+1+(j-1)*4,(1:num_q) + 1+nX+nU+nL+2*nC+nClutch) = 2*phi(2*j)*dPhi(2*j,:);
%         end
        
        % use a + b - sqrt(a^2 + b^2) instead of complimentarity
        % This does not seem great, essentially asymptotic behavior near
        % alpha
        % This function is also called the Fischer-Burmeister function
%         d_i(nX+2*nL+1+(j-1)*4) = (cLambda(2*j,i+1) + phi(2*j) - sqrt(cLambda(2*j,i+1)^2 + phi(2*j)^2))*contactMult;
%         dd_i(nX+2*nL+1+(j-1)*4,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j) = (1 - cLambda(2*j,i+1)/sqrt(cLambda(2*j,i+1)^2 + phi(2*j)^2))*contactMult;
%         dd_i(nX+2*nL+1+(j-1)*4,(1:num_q) + 1+nX+nU+nL+2*nC+nClutch) = (dPhi(2*j,:) * (1 - phi(2*j)/sqrt(cLambda(2*j,i+1)^2 + phi(2*j)^2)))*contactMult;

        mu = 1;
        % (xforce-mu*zforce)*(xforce+mu*zforce)*z-force*x-velocity = 0 
        % x-vel must be zero if z-force is non-zero and the x-force is
        % not at friction limits
        % This equation needs to be updated for friction limits
        
        % this is going to be psi*(mu^2 lambda_z^2 - lambda_x^2) = 0
        d_i(nX+2*nL+2+(j-1)*5) = contactMult * psi(j) * (mu^2*cLambda(2*j,i+1)^2 - cLambda(2*j-1,i+1)^2);
        dd_i(nX+2*nL+2+(j-1)*5,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j) = contactMult * psi(j) * 2*mu^2*cLambda(2*j,i+1);
        dd_i(nX+2*nL+2+(j-1)*5,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j-1) = -2 * contactMult * psi(j) *cLambda(2*j-1,i+1);
        dd_i(nX+2*nL+2+(j-1)*5,1+nX+nU+nL+2*nC+nClutch+(1:nX)) = contactMult * dPsi(j,:) * (mu^2*cLambda(2*j,i+1)^2 - cLambda(2*j-1,i+1)^2);        

        %2 inequality constraints 0 <= d <= inf
        d_i(nX+2*nL+3+(j-1)*5) = phi(2*j);  %zheight >= 0
        dd_i(nX+2*nL+3+(j-1)*5,(1:num_q) + 1+nX+nU+nL+2*nC+nClutch) = dPhi(2*j,:);
        
        
%         account for sliding friction possibility?
        d_i(nX+2*nL+4+(j-1)*5) = mu^2*cLambda(2*j,i+1)^2 - cLambda(2*j-1,i+1)^2;  %friction limit
        dd_i(nX+2*nL+4+(j-1)*5,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j) = 2*mu^2*cLambda(2*j,i+1);
        dd_i(nX+2*nL+4+(j-1)*5,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j-1) = -2*cLambda(2*j-1,i+1);
        
        %one more inequality, psi*lambdax <= 0
        d_i(nX+2*nL+5+(j-1)*5) = -cLambda(2*j-1,i+1) * psi(j);
        dd_i(nX+2*nL+5+(j-1)*5,1+2*nX+2*nU+2*nL+2*nC+nClutch+2*j-1) = -psi(j);
        dd_i(nX+2*nL+5+(j-1)*5,1+nX+nU+nL+2*nC+nClutch+(1:nX)) = -cLambda(2*j-1,i+1) * dPsi(j,:);
      end
    end
    
    %Unilateral position constraints
    if nL > 0
      nJointLimitConst = sys.getNumJointLimitConstraints;
      jointLimitMult = 10;
      % Handle Joint Limit constraints first
      if (nJointLimitConst > 0)
        [phi,J] = sys.jointLimitConstraints(x(1:num_q,i+1));
        for j=1:nJointLimitConst,
          d_i(nX+1+(j-1)*2) = lambda(j,i+1) * phi(j) * jointLimitMult;
          dd_i(nX+1+(j-1)*2,1+2*nX+2*nU+nL+2*nC+nClutch+j) = phi(j) * jointLimitMult;
          dd_i(nX+1+(j-1)*2,(1:num_q)+1+nX+nU+nL+2*nC+nClutch) = lambda(j,i+1) * J(j,:) * jointLimitMult;
          
          d_i(nX+2+(j-1)*2) = phi(j);
          dd_i(nX+2+(j-1)*2,(1:num_q)+1+nX+nU+nL+2*nC+nClutch) = J(j,:);
        end
      end
      
      if (nL - nJointLimitConst) > 0
        % Handle position equality constraints
        eqMult = 1;
        [phi,J] = sys.positionConstraints(x(1:num_q,i+1));
        d_i(nX+nJointLimitConst*2+(1:nL-nJointLimitConst)) = phi*eqMult;
        dd_i(nX+nJointLimitConst*2+(1:nL-nJointLimitConst),(1:num_q)+1+nX+nU+nL+2*nC+nClutch) = J*eqMult;
      end
    end

    if (bxc)
      [c,dc] = geval(con.x.c,x(:,i),options);
      fxc = [fxc; c(:)]; Gxc = [Gxc; dc(:)];
    end
    
        %Evaluate the cost function at the knot point i and gradient
%     [g(i),dg_i] = geval(costFun,dt(i),x(:,i),u(:,i+1),options);
    [g(i),dg_i] = costFun(dt(i),x(:,i),u(:,i+1),sys);
    dg_i(1,1) = dg_i(1,1)*ddtdw1(i);
    dg(1,:,i) = dg_i;
    
    d(:,i) = d_i;
    dd(:,:,i) = dd_i;
  end
  
  %endpoint position constraint
  d_end = zeros(2*nL,1);
  dd_end = zeros(2*nL,1+nX+nU+nL+2*nC);
  
  %Initial position constraints
%   doKinematicsAndVelocities(sys.model,x(1:num_q,1),x(num_q+1:2*num_q));

%  kinsol = doKinematics(sys,x(1:num_q,1));

  %Unilateral position constraints
  if nL > 0
      nJointLimitConst = sys.getNumJointLimitConstraints;
      % Handle Joint Limit constraints first
      if (nJointLimitConst > 0)
        [phi,J] = sys.jointLimitConstraints(x(1:num_q,1));
        for j=1:nJointLimitConst,
          d_end(1+(j-1)*2) = lambda(j,1) * phi(j);
          dd_end(1+(j-1)*2,1+nX+nU+j) = phi(j);
          dd_end(1+(j-1)*2,(1:num_q)+1) = lambda(j,1) * J(j,:);
          
          d_end(2+(j-1)*2) = phi(j);
          dd_end(2+(j-1)*2,(1:num_q)+1) = J(j,:);
        end
      end
      
      if (nL - nJointLimitConst) > 0
        % Handle position equality constraints
        [phi,J] = sys.positionConstraints(x(1:num_q,1));
        d_end(nJointLimitConst*2+(1:nL-nJointLimitConst)) = phi;
        dd_end(nJointLimitConst*2+(1:nL-nJointLimitConst),(1:num_q)+1) = J;
      end
    end
    
%   if nL > 0
%     [phi,J] = geval(@sys.positionConstraints,x(1:num_q,nT));
%     d_end = phi;
%     dd_end = [zeros(nL,1) J zeros(nL,num_q+nU+nL)];
%   elseif nC > 0
%     d_end = [zeros(4*nC,1)];
%     dd_end = [zeros(4*nC,1+nX+nU+nL+2*nC)];
%   else
%     d_end = [];
%     dd_end = [];
%   end
  [h,dh] = geval(finalCostFun,t(end),x(:,end),options);

  J = h + sum(g);
  dJ = [dh(1)*ddtdw1'+reshape(dg(1,1,:),1,[]), reshape(dg(1,1+(1:nX),:),1,[]), dh(1,2:end),zeros(1,nU),reshape(dg(1,1+nX+(1:nU),:),1,[])];
  
  % Add cost to tscale fvariance
  tscale = w(1:nT-1);
  mean_tscale = mean(tscale);
  R_tscale = 100;
  variance = sum((tscale - mean_tscale).^2);
  g_var= R_tscale*variance/(mean_tscale^2);
  dg_var = R_tscale*2*(tscale - mean_tscale)/(mean_tscale^2) + ...
      -(2/(nT-1))*R_tscale*variance/(mean_tscale^3);
  
  J = J + g_var;
  dJ(1:nT-1) = dJ(1:nT-1) + dg_var';

  if (isfield(options,'trajectory_cost_fun') && ~isempty(options.trajectory_cost_fun))
    [Jtraj,dJtraj]=geval(options.trajectory_cost_fun,t,x,u,options);
    J = J+Jtraj;
    dJ(1) = dJ(1)+dJtraj(1:nT)*dtdw1;
    dJ(2:end) = dJ(2:end)+dJtraj(nT-1+1:end);
  end
  
%   d_time = t(end)-t(1);
%   dd_time = ddtdw1;
%   f = [J; d_time(:); d(:); d_end(:)];
%   G = [dJ(:); dd_time(:); dd(:); dd_end(:)];
% replaced time with a linear constraint below
f = [J; 0; d(:); d_end(:)];
G = [dJ(:); dd(:); dd_end(:)];

  if (bxc)
    % add xf
    [c,dc] = geval(con.x.c,x(:,end),options);
    fxc = [fxc; c(:)]; Gxc = [Gxc; dc(:)];

    % put everything into the main f and G
    f = [f; fxc]; 
    G = [G; Gxc]; 
  end
  
  if (isfield(con,'xf'))
    if (isfield(con,'c'))
      [c,dc] = geval(con.xf.c,x(:,end),options);
      f = [f; c(:)]; G = [G; dc(:)];
    end
    if (isfield(con.xf,'ceq'))
      [c,dc] = geval(con.xf.ceq,x(:,end),options);
      f = [f; c(:)]; G = [G; dc(:)];
    end
  end
  
  if (isfield(con,'x0'))
    if (isfield(con,'c'))
      [c,dc] = geval(con.x0.c,x(:,1),options);
      f = [f; c(:)]; G = [G; dc(:)];
    end
    if (isfield(con.x0,'ceq'))
      [c,dc] = geval(con.x0.ceq,x(:,1),options);
      f = [f; c(:)]; G = [G; dc(:)];
    end
  end

  if (isfield(con,'periodic') && con.periodic)
    f = [f;zeros(nX+nU,1)];  % implemented as linear constraint below
  end
  
  if (isfield(con,'fixtime') && con.fixtime == 1)
    f = [f;zeros(nT-2,1)];  % implemented as linear constraint below
  elseif (isfield(con,'fixtime') && con.fixtime == 2)
    f = [f;zeros(floor((nT-1)/2)-1,1)];  % implemented as linear constraint below
  end
  
  if (isfield(con,'noflight') && con.noflight)
    f = [f;zeros(nT,1)];  % implemented as linear constraint below
  end
  
  if (isfield(con,'linperiodic'))
    f = [f;zeros(sum(sum(abs(con.linperiodic),2) > 0),1)];  % implemented as linear constraint below
  end
  
  if any(G(options.grad_skip) ~= 0)
    error('Whoops, somehow tried to skip a non-zero gradient element')
  end
  
  G = G(options.grad_I);
  
%  pause(0);
end

function [nf, A, iAfun, jAvar, iGfun, jGvar, Fhigh, Flow, oname] = userfun_grad_ind(sys,nT,nX,nU,nL,nC,nClutch,con,x,t,options)
  % dJ:
  nf = 1;
  A = []; iAfun = []; jAvar = [];
  iGfun = repmat(1,nT-1+nX*nT+nU*nT,1);  
  jGvar = (1:(nT-1+nX*nT+nU*nT))';
  oname{1} = 'J';
  
  Fhigh = inf;  Flow = -inf;
  
  %Time constraint
  A = [A; reshape(diff(t),nT-1,1)];
  iAfun = [iAfun; repmat(2,nT-1,1)];
  jAvar = [jAvar;(1:nT-1)'];
%   iGfun = [iGfun;repmat(2,nT-1,1)];
%   jGvar = [jGvar;(1:(nT-1))'];
  Fhigh = [Fhigh; con.T.ub - t(1)];%/(t(end)-t(1))];
  Flow = [Flow; con.T.lb - t(1)];%/(t(end)-t(1))];
  
  nf = nf + 1;

      % transcription constraints:
  for i=1:(nT-1)
    iGfun = [iGfun; nf+reshape(repmat(1:(nX+2*nL+5*nC),(1+2*nX+2*nU+2*nL+4*nC+2*nClutch),1)',[],1)];
    jGvar = [jGvar; reshape(repmat([...
      i,...
      nT-1+nX*(i-1)+(1:nX),...
      nT-1+nX*nT+nU*(i-1)+(1:nU),...
      nT-1+nX*nT+nU*nT+nL*(i-1)+(1:nL),...
      nT-1+nX*nT+nU*nT+nL*nT+2*nC*(i-1)+(1:2*nC),...
      nT-1+nX*nT+nU*nT+nL*nT+2*nC*nT+nClutch*(i-1)+(1:nClutch),...
      nT-1+nX*i+(1:nX),...
      nT-1+nX*nT+nU*i+(1:nU),...
      nT-1+nX*nT+nU*nT+nL*i+(1:nL),...
      nT-1+nX*nT+nU*nT+nL*nT+2*nC*i+(1:2*nC),...
      nT-1+nX*nT+nU*nT+nL*nT+2*nC*nT+nClutch*i+(1:nClutch)],...
      nX+2*nL+5*nC,1),[],1)];
    nf = nf+nX+2*nL+5*nC;
    
    if (nargout>8)
      error('Second collocation constraint not updated for oname');
      for j=1:nX, oname= {oname{:},['d_',num2str(j),'(',num2str(i),')']}; end
    end
  end
  alpha = .1*con.alphamult;
  beta = .01*con.betamult;
  %when switching to real friction limits, 0,0,inf,inf
  %[alpha,inf] for joint limits
  nJointLimitConst = sys.getNumJointLimitConstraints;
  Fhigh = [Fhigh; repmat([zeros(nX,1);repmat([alpha;inf],nJointLimitConst,1);repmat([0;0],nL-nJointLimitConst,1);repmat([alpha;beta;inf;inf;inf],nC,1)],nT-1,1)];
  Flow = [Flow; repmat([zeros(nX,1);repmat([0;0],nL,1);repmat([0;-beta;0;0;0],nC,1)],nT-1,1)];

  %Add initial state position constraints
  iGfun = [iGfun; nf+reshape(repmat(1:(2*nL),(1+nX+nU+nL+2*nC),1)',[],1)];
  jGvar = [jGvar; reshape(repmat([1,nT-1+(1:nX),nT-1+nX*nT+(1:nU),...
    nT-1+nX*nT+nU*nT+(1:nL),nT-1+nX*nT+nU*nT+nL*nT+(1:2*nC)],2*nL,1),[],1)];
  nf = nf + 2*nL;
  Fhigh = [Fhigh; repmat([alpha;inf],nJointLimitConst,1); repmat([0;0],nL-nJointLimitConst,1)];
  Flow = [Flow; repmat([0;0],nL,1)];
  
  %Add endpoint position and contact constraints
%   i=nT;
%   iGfun = [iGfun; nf+reshape(repmat(1:(2*nL+2*nC),(1+nX+nU+nL+2*nC),1)',[],1)];
%   jGvar = [jGvar; reshape(repmat([nT-1,nT-1+nX*(i-1)+(1:nX),nT-1+nX*nT+nU*(i-1)+(1:nU),...
%     nT-1+nX*nT+nU*nT+nL*(i-1)+(1:nL),nT-1+nX*nT+nU*nT+nL*nT+2*nC*(i-1)+(1:2*nC)],nL+4*nC,1),[],1)];
%   nf = nf+2*nL+4*nC;
%   
%   Fhigh = [Fhigh;zeros(nL,1);repmat([alpha;beta;inf;inf],nC,1)];
%   Flow = [Flow;zeros(nL,1);repmat([0;-beta;0;0],nC,1)];

  if (isfield(con,'x'))
    if (isfield(con.x,'c'))
      c = feval(con.x.c,x(:,end));
      n=length(c);
      for i=1:nT
        iGfun = [iGfun; nf+reshape(repmat((1:n)',1,nX),[],1)];
        jGvar = [jGvar; nT-1+nX*(i-1)+reshape(repmat(1:nX,n,1),[],1)];
        Fhigh = [Fhigh;zeros(n,1)];
        Flow = [Flow;repmat(-inf,n,1)];
        if (nargout>5)
          for j=1:nX, oname= {oname{:},['con.x(',num2str(i),').c_',num2str(j)]}; end
        end
        nf = nf+n;
      end
    end
  end
  
  if (isfield(con,'xf'))
    if (isfield(con.xf,'c'))
      c = feval(con.xf.c,x(:,end));
      n=length(c);
      iGfun = [iGfun; nf+reshape(repmat((1:n)',1,nX),[],1)];
      jGvar = [jGvar; nT-1+nX*(nT-1)+reshape(repmat(1:nX,n,1),[],1)];
      Fhigh = [Fhigh;zeros(n,1)];
      Flow = [Flow;repmat(-inf,n,1)];
      if (nargout>5)
        for j=1:nX, oname= {oname{:},['con.xf.c_',num2str(j)]}; end
      end
      nf = nf+n;
    end
    if (isfield(con.xf,'ceq'))
      c = feval(con.xf.ceq,x(:,end));
      n=length(c);
      iGfun = [iGfun; nf+reshape(repmat((1:n)',1,nX),[],1)];
      jGvar = [jGvar; nT-1+nX*(nT-1)+reshape(repmat(1:nX,n,1),[],1)];
      Fhigh = [Fhigh;zeros(n,1)];
      Flow = [Flow;zeros(n,1)];
      if (nargout>5)
        for j=1:nX, oname= {oname{:},['con.xf.ceq_',num2str(j)]}; end
      end
      nf = nf+n;
    end
  end
  
  if (isfield(con,'x0'))
    if (isfield(con.x0,'c'))
      c = feval(con.x0.c,x(:,1));
      n=length(c);
      iGfun = [iGfun; nf+reshape(repmat((1:n)',1,nX),[],1)];
      jGvar = [jGvar; nT-1+reshape(repmat(1:nX,n,1),[],1)];
      Fhigh = [Fhigh,zeros(n,1)];
      Flow = [Flow,repmat(-inf,n,1)];
      if (nargout>5)
        for j=1:nX, oname= {oname{:},['con.x0.c_',num2str(j)]}; end
      end
      nf = nf+n;
    end
    if (isfield(con.x0,'ceq'))
      c = feval(con.x0.ceq,x(:,1));
      n=length(c);
      iGfun = [iGfun; nf+reshape(repmat((1:n)',1,nX),[],1)];
      jGvar = [jGvar; nT-1+reshape(repmat(1:nX,n,1),[],1)];
      Fhigh = [Fhigh;zeros(n,1)];
      Flow = [Flow;zeros(n,1)];
      if (nargout>5)
        for j=1:nX, oname= {oname{:},['con.x0.ceq_',num2str(j)]}; end
      end
      nf = nf+n;
    end
  end
  
  if (isfield(con,'periodic') && con.periodic)
    % then add linear constraints  x0[i]=xf[i] and u0[i]=uf[i]
    A = [A; repmat([1;-1],nX+nU,1)];
    iAfun = [iAfun; nf+reshape(repmat(1:(nX+nU),2,1),[],1)];
    x0ind = nT-1+(1:nX); xfind = nT-1+nX*(nT-1) + (1:nX);
    u0ind = nT-1+nX*nT-1+(1:nU); ufind = nT-1+nX*nT+nU*(nT-1) + (1:nU);   
    jAvar = [jAvar; reshape([x0ind,u0ind; xfind,ufind],[],1)];
    Fhigh = [Fhigh; zeros(nX+nU,1)];
    Flow = [Flow; zeros(nX+nU,1)];
    if (nargout>5)
      for j=1:nX, oname= {oname{:},['con.periodic_x',num2str(j)]}; end
      for j=1:nU, oname= {oname{:},['con.periodic_u',num2str(j)]}; end
    end      
    nf = nf + nX+nU;  
  end
  
  
  %add constraints to fix dt(i) = dt(j)
  if (isfield(con,'fixtime') && con.fixtime == 1)
    for i=1:nT-2,
      A = [A; 1; -1];
      iAfun = [iAfun; nf + 1; nf + 1];
      jAvar = [jAvar; i; i + 1];
      nf = nf + 1;
    end
    Fhigh = [Fhigh; zeros(nT-2,1)];
    Flow = [Flow; zeros(nT-2,1)];
  elseif (isfield(con,'fixtime') && con.fixtime == 2)
    ndtPairs = floor((nT-1)/2);
    for i=1:ndtPairs-1,
      A = [A; 1; 1; -1; -1];
      iAfun = [iAfun; repmat(nf + 1,4,1)];
      jAvar = [jAvar; 2*i - 1; 2*i;2*i + 1; 2*i+2];
      nf = nf + 1;
    end
    Fhigh = [Fhigh; zeros(ndtPairs-1,1)];
    Flow = [Flow; zeros(ndtPairs-1,1)];
  end
  
  if (isfield(con,'noflight') && con.noflight)
    for i=1:nT,
      A = [A; ones(nC,1)];
      iAfun = [iAfun; (nf+1)*ones(nC,1)];
      jAvar = [jAvar; nT-1+nX*nT+nU*nT+nL*nT+2*nC*(i-1)+(2:2:2*nC)'];
      nf = nf + 1;
    end
    Fhigh = [Fhigh; inf(nT,1)];
    Flow = [Flow; .1*ones(nT,1)];
  end
  
  if isfield(con,'linperiodic')
    x0ind = nT-1+(1:nX); xfind = nT-1+nX*(nT-1) + (1:nX);
    u0ind = nT-1+nX*nT+(1:nU); ufind = nT-1+nX*nT+nU*(nT-1) + (1:nU);
    for i=1:size(con.linperiodic,1),
      if sum(abs(con.linperiodic(i,:))) > 0
        A = [A; -1];
        iAfun = [iAfun; nf + 1];
        if i > nX
          jAvar = [jAvar; u0ind(i-nX)];
        else
          jAvar = [jAvar; x0ind(i)];
        end
        for j=1:size(con.linperiodic,2),
          if (con.linperiodic(i,j) ~= 0)
            A = [A; con.linperiodic(i,j)];
            iAfun = [iAfun; nf + 1];
            if j > nX
              jAvar = [jAvar; ufind(j-nX)];
            else
              jAvar = [jAvar; xfind(j)];
            end
          end
        end
        nf = nf + 1;
        if isfield(con,'linperiodic_min')
          Flow = [Flow; con.linperiodic_min(i)];
        else
          Flow = [Flow; 0];
        end
        if isfield(con,'linperiodic_max')
          Fhigh = [Fhigh; con.linperiodic_max(i)];
        else
          Fhigh = [Fhigh; 0];
        end
      end    
    end
  end
  
  if (isfield(con,'periodic') && con.periodic)
    A = [A; repmat([-1; 1],nT-2,1)];
    iAfun = [iAfun; reshape(repmat(nf+1:nf+nT-2,2,1),[],1)];
    jAvar = [jAvar; reshape([1:nT-2;2:nT-1],[],1)];
    nf = nf + nT-2;
    Fhigh = [Fhigh; zeros(nT-2,1)];
    Flow = [Flow; zeros(nT-2,1)];
  end
end


function [utraj,traj,trans_info] = dirtran_wrapup(sys,w,t,nX,nU,nL,nC,nClutch,options)
tOrig = t;
nT = length(t);
t = tOrig(1);
I = 1;
j = 1;
for i=1:nT-1  
  if w(i) > 0
    t(j+1) = t(j) + (tOrig(j+1) - tOrig(j))*w(i);
    I = [I;i+1];
    j = j+1;
  end
end


x = reshape(w(nT-1+ (1:nT*nX)),nX,nT);
u = reshape(w((nT-1+ nT*nX + 1):(nT-1+nT*nX+nT*nU)),nU,nT);
lambda = reshape(w((nT-1+nT*(nX+nU)+1):(nT-1+nT*(nX+nU+nL))),nL,nT);
cLambda = reshape(w((nT-1+nT*(nX+nU+nL)+1):(nT-1+nT*(nX+nU+nL+2*nC))),2*nC,nT);
clutch = reshape(w((nT+nT*(nX+nU+nL+2*nC)):(nT-1+nT*(nX+nU+nL+2*nC+nClutch))),nClutch,nT);
lambda = [lambda;cLambda;clutch];

x = reshape(w(nT-1+[1:(nT*nX)]),nX,nT);
u = reshape(w((nT+nT*nX):(nT-1+nT*nX+nT*nU)),nU,nT);

x = x(:,I);
u = u(:,I);
lambda = lambda(:,I);
if nU > 0
  utraj = PPTrajectory(foh(t,[u(:,2) u(:,2:end)]));
% utraj = PPTrajectory(foh(t,u));
  utraj = utraj.setOutputFrame(sys.getInputFrame);
else 
  utraj = [];
end
if (nC + nL == 0)
  ltraj = [];
else
  ltraj = PPTrajectory(foh(t,lambda));
end
xtraj = PPTrajectory(foh(t,x));
xtraj = xtraj.setOutputFrame(sys.getStateFrame);
% for i=1:nT
%   xdot(:,i) = sys.dynamics(t(i),x(:,i),u(:,i));
% end
% xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
traj.xtraj = xtraj;
traj.ltraj = ltraj;

trans_info.grad_I = options.grad_I;
trans_info.grad_skip = options.grad_skip;

end

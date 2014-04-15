function [w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,userfun,wrapupfun,iname,oname] = dircolSNOPTtranscription(sys,costFun,finalCostFun,x0,utraj0,con,options)
% Direct collocation method.  
%  This function should not be called directly.  Use the
%  trajectoryOptimization interface. 
%
%  I use the algorithm as described in Enright91 and Hargraves86
%  Basic algorithm:  
%    u(t) is represented as a first-order spline
%    xc(t) is represented as a cubic spline
%    xd(t) is represented as a zero-order spline (zoh)
%  At every knot point, add a constraint enforcing the derivatives and
%  updates to match the spline derivatives, etc.
%
%  Since I don't know many details about the model (e.g. hybrid events
%  can be hidden), this implementation assumes things are smooth and tries
%  to power through unknown difficulties.  
%

if (~isfield(options,'xtape0')), options.xtape0='free'; end

nU = sys.getNumInputs();
nX = sys.getNumContStates();
nXd = sys.getNumDiscStates();
if (nXd>0)
  error('not implemented yet (but should be very straightforward)');
end

ts = sys.getSampleTime();
if (size(ts,2)>1 || any(ts~=0))
  warning('found non-continuous sample times.  i''ll proceed assuming everything is continuous'); 
%  error('not implemented yet (but should be very straightforward)');
end

t = utraj0.getBreaks(); t=t(:);
tscale = 1;  % stretch time by this amount
u = utraj0.eval(t);
if (strcmp(options.xtape0,'simulate'))
  drivensys = cascade(utraj0,sys);
  xtraj = simulate(drivensys,[t(1),t(end)],x0);
  x = xtraj.eval(t);
else % options.xtape0='rand' or 'free'
  x = repmat(x0,1,length(t));%+.1*randn(length(x0),length(t));   x(:,1) = x0;
end
  
nT = length(t);

%% Set up snopt inputs
w0 = [tscale; x(:); u(:)];
whigh = [ tscale; repmat(inf,prod(size(x)),1); repmat(inf,prod(size(u)),1)];
wlow = [ tscale; repmat(-inf,prod(size(x)),1); repmat(-inf,prod(size(u)),1)];
A=[];
iAfun=[];
jAvar=[];

% handle constraints
  function conwarn(f1,f2)
    if (iscell(f1)) f1 = f1{1}; end
    if (nargin>1)
      if (iscell(f2)) f2 = f2{1}; end
      warning([f1,'.',f2,' constraint is not supported by dircol (at least not yet)']);
    else
      warning([f1,' constraint is not supported by dircol (at least not yet)']);
    end      
  end

for f1 = fieldnames(con)'
  switch(f1{1})
    case 'x'
      xind=1+(1:prod(size(x)));
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
    case 'x0'
      for f2 = fieldnames(con.x0)'
        switch(f2{1})
          case 'lb'
            wlow(1 + (1:nX)) = max(wlow(1 + (1:nX)),con.x0.lb);
          case 'ub'
            whigh(1 + (1:nX)) = min(whigh(1 + (1:nX)),con.x0.ub);
          case {'c','ceq'}
            % intentionally pass through.. these get handled in the userfun
          otherwise 
            conwarn(f1,f2);
        end
      end
    case 'xf'
      xfind = 1 + nX*(nT-1) + (1:nX);
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
        w0(1+(1:(nT*nX))) = x(:);
      end
    case 'u'
      for f2 = fieldnames(con.u)'
        uind = 1 + nX*nT + 1 : length(wlow);
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
        uind = 1 + nX*nT + (1 : nU);
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
        uind = 1 + nX*nT + nU*(nT-1) + (1 : nU);
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
      for f2 = fieldnames(con.T)'
        switch(f2{1})
          case 'lb'
            wlow(1) = con.T.lb/(t(end)-t(1));
          case 'ub'
            whigh(1) = con.T.ub/(t(end)-t(1));
          otherwise 
            conwarn(f1,f2);
        end
      end
      
    case 'periodic'
      % intentionally pass through.  this is implemented in userfun_ind
      
    otherwise
      conwarn(f1);
  end
end

userfun = @(w) dircol_userfun(sys,w,costFun,finalCostFun,t,nX,nU,con,options);
wrapupfun = @(w) dircol_wrapup(sys,w,t,nX,nU);

if (options.grad_test)
  % print out the debugging key:
  iname = {'tscale'};
  for i=1:nT, for j=1:nX, iname= {iname{:},['x_',num2str(j),'(',num2str(i),')']}; end, end
  for i=1:nT, for j=1:nU, iname= {iname{:},['u_',num2str(j),'(',num2str(i),')']}; end, end
  
  [nf,A,iAfun,jAvar,iGfun,jGvar,Fhigh,Flow,oname] = userfun_grad_ind(nT,nX,nU,con,x,options);
else
  [nf,A,iAfun,jAvar,iGfun,jGvar,Fhigh,Flow] = userfun_grad_ind(length(t),nX,nU,con,x,options);
  iname={};
  oname={};
end

end

function [f,G] = dircol_userfun(sys,w,costFun,finalCostFun,tOrig,nX,nU,con,options)
  nT = length(tOrig);
  t = w(1)*tOrig;          dtdw1 = tOrig;
  dt = w(1)*diff(tOrig);   ddtdw1 = diff(tOrig);
  x = reshape(w(1+[1:(nT*nX)]),nX,nT);
  u = reshape(w((2+nT*nX):end),nU,nT);

  tcol = t(1:end-1)+dt/2;  dtcoldw1 = dtdw1(1:end-1)+ddtdw1/2;
  ucol = .5*(u(:,1:end-1)+u(:,2:end));  
  
%  figure(1); clf; plot(x(1,:),x(2,:)); drawnow;

  
  %% todo: vectorize this when possible

  % preallocate vars
  g = zeros(1,nT);
  dg = zeros(1,1+nX+nU,nT);
  xdot = zeros(nX,nT);
  dxdot = zeros(nX,1+nX+nU,nT);
  d = zeros(nX,nT-1);
  dd = zeros(nX,1+2*nX+2*nU,nT-1);
  J=0;
  dJ = zeros(1,1+nX*nT+nU*nT);
  
  bxc = isfield(con,'x') && isfield(con.x,'c');  % eval boolean once (not repititively inside the loop)
  fxc = [];  Gxc=[];
  
  % iterate through time
  [xdot(:,1),dxdot(:,:,1)] = geval(@sys.dynamics,t(1),x(:,1),u(:,1),options);  
  dxdot(:,1,1) = dxdot(:,1,1)*dtdw1(1); % d/d[tscale; x(:,1); u(:,1)]
  [g(1),dg(1,:,1)] = geval(costFun,t(1),x(:,1),u(:,1),options);  dg(1,1,1)=dg(1,1,1)*dtdw1(1);  % d/d[tscale; x(:,1); u(:,1)]
  for i=1:(nT-1)  % compute dynamics and cost
    [g(i+1),dg(1,:,i+1)] = geval(costFun,t(i+1),x(:,i+1),u(:,i+1),options);  dg(1,1,i+1)=dg(1,1,i+1)*dtdw1(i+1);  % d/d[tscale; x(:,i+1); u(:,i+1)]
    [xdot(:,i+1),dxdot(:,:,i+1)] = geval(@sys.dynamics,t(i+1),x(:,i+1),u(:,i+1),options);
    dxdot(:,1,1)=dxdot(:,1,1)*dtdw1(i+1); % d/d[tscale; x(:,i+1); u(:,i+1)]
    xcol = .5*(x(:,i)+x(:,i+1)) + dt(i)/8*(xdot(:,i)-xdot(:,i+1));
    dxcol = .5*[zeros(nX,1),eye(nX),zeros(nX,nU),eye(nX),zeros(nX,nU)] + ...
      [ddtdw1(i)/8*(xdot(:,i)-xdot(:,i+1)), zeros(nX,nX+nU+nX+nU)] + ...
      dt(i)/8*[dxdot(:,:,i),zeros(nX,nX+nU)] + ...
      -dt(i)/8*[dxdot(:,1,i+1),zeros(nX,nX+nU),dxdot(:,2:end,i+1)]; % d/d[tscale;x(:,i);u(:,i);x(:,i+1);u(:,i+1)]
    xdotcol = -1.5*(x(:,i)-x(:,i+1))/dt(i) - .25*(xdot(:,i)+xdot(:,i+1));
    dxdotcol = -1.5*[-(x(:,i)-x(:,i+1))*ddtdw1(i)/(dt(i)^2), eye(nX)/dt(i), zeros(nX,nU), -eye(nX)/dt(i), zeros(nX,nU)] + ...
      -.25*[dxdot(:,:,i),zeros(nX,nX+nU)] + ...
      -.25*[dxdot(:,1,i+1),zeros(nX,nX+nU),dxdot(:,2:end,i+1)]; % d/d[tscale;x(:,i);u(:,i);x(:,i+1);u(:,i+1)]
    % collocation constraint:
    [d(:,i),df]= geval(@sys.dynamics,tcol(i),xcol,ucol(:,i),options);
    d(:,i) = d(:,i) - xdotcol;
    dd(:,:,i) = df*[dtcoldw1(i),zeros(1,2*nX+2*nU); dxcol; zeros(nU,1+nX), .5*eye(nU), zeros(nU,nX), .5*eye(nU)] - dxdotcol;  % d/d[tscale;x(:,i);u(:,i);x(:,i+1);u(:,i+1)]

    if (bxc)
      [c,dc] = geval(con.x.c,x(:,i),options);
      fxc = [fxc; c(:)]; Gxc = [Gxc; dc(:)];
    end
    
    J = J + dt(i)*(g(i)+g(i+1))/2; % trapezoidal rule
    dJ(1) = dJ(1) + ddtdw1(i)*(g(i)+g(i+1))/2 + dt(i)*(dg(1,1,i)+dg(1,1,i+1))/2;
    xi_inds=1+(i-1)*nX+(1:nX);
    dJ(xi_inds) = dJ(xi_inds) + dt(i)*dg(1,1+(1:nX),i)/2;
    xip_inds = xi_inds+nX;
    dJ(xip_inds) = dt(i)*dg(1,1+(1:nX),i+1)/2;
    ui_inds = 1+nT*nX+(i-1)*nU+(1:nU);
    dJ(ui_inds) = dJ(ui_inds) + dt(i)*dg(1,1+nX+(1:nU),i)/2;
    uip_inds = ui_inds+nU; 
    dJ(uip_inds) = dt(i)*dg(1,1+nX+(1:nU),i+1)/2;
    
    % for debugging
%    d(:,i)=xdotcol;
%    dd(:,:,i)=dxdotcol;
  end
  [h,dh] = geval(finalCostFun,t(end),x(:,end),options); dh(1)=dh(1)*dtdw1(end);

  J = J + h;
  dJ(1) = dJ(1)+dh(1);
  xf_inds = 1+(nT-1)*nX+(1:nX);
  dJ(xf_inds) = dJ(xf_inds) + dh(2:end); 
  
  if (isfield(options,'trajectory_cost_fun') && ~isempty(options.trajectory_cost_fun))
    [Jtraj,dJtraj]=geval(options.trajectory_cost_fun,t,x,u,options);
    J = J+Jtraj;
    dJ(1) = dJ(1)+dJtraj(1:nT)*dtdw1;
    dJ(2:end) = dJ(2:end)+dJtraj(nT+1:end);
  end
  if isfield(options,'plan_publisher')
    options.plan_publisher.publish(t,x);  % todo: also publish u
  end
  
  f = [J; d(:)];
  G = [dJ(:); dd(:)];

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

end

function [nf, A, iAfun, jAvar, iGfun, jGvar, Fhigh, Flow, oname] = userfun_grad_ind(nT,nX,nU,con,x,options)

  % dJ:
  nf = 1;
  A = []; iAfun = []; jAvar = [];
  iGfun = repmat(1,1+nX*nT+nU*nT,1);  
  jGvar = (1:(1+nX*nT+nU*nT))';
  oname{1} = 'J';
  
  Fhigh = inf;  Flow = -inf;
  
  % collocation constraints:
  for i=1:(nT-1)
    iGfun = [iGfun; nf+reshape(repmat(1:nX,(1+2*nX+2*nU),1)',[],1)];
    jGvar = [jGvar; reshape(repmat([1,1+nX*(i-1)+(1:nX),1+nX*nT+nU*(i-1)+(1:nU),1+nX*i+(1:nX),1+nX*nT+nU*i+(1:nU)],nX,1),[],1)];
    nf = nf+nX;
    if (nargout>5)
      for j=1:nX, oname= {oname{:},['d_',num2str(j),'(',num2str(i),')']}; end
    end
  end    
  
  Fhigh = [Fhigh;zeros(nf-1,1)];
  Flow = [Flow;zeros(nf-1,1)];

  if (isfield(con,'x'))
    if (isfield(con.x,'c'))
      c = feval(con.x.c,x(:,end));
      n=length(c);
      for i=1:nT
        iGfun = [iGfun; nf+reshape(repmat((1:n)',1,nX),[],1)];
        jGvar = [jGvar; 1+nX*(i-1)+reshape(repmat(1:nX,n,1),[],1)];
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
      jGvar = [jGvar; 1+nX*(nT-1)+reshape(repmat(1:nX,n,1),[],1)];
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
      jGvar = [jGvar; 1+nX*(nT-1)+reshape(repmat(1:nX,n,1),[],1)];
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
      jGvar = [jGvar; 1+reshape(repmat(1:nX,n,1),[],1)];
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
      jGvar = [jGvar; 1+reshape(repmat(1:nX,n,1),[],1)];
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
    x0ind = 1+(1:nX); xfind = 1+nX*(nT-1) + (1:nX);
    u0ind = 1+nX*nT+(1:nU); ufind = 1+nX*nT+nU*(nT-1) + (1:nU);   
    jAvar = [jAvar; reshape([x0ind,u0ind; xfind,ufind],[],1)];
    Fhigh = [Fhigh; zeros(nX+nU,1)];
    Flow = [Flow; zeros(nX+nU,1)];
    if (nargout>5)
      for j=1:nX, oname= {oname{:},['con.periodic_x',num2str(j)]}; end
      for j=1:nU, oname= {oname{:},['con.periodic_u',num2str(j)]}; end
    end      
    nf = nf + nX+nU;  
  end
end


function [utraj,xtraj] = dircol_wrapup(sys,w,t,nX,nU)

t = t*w(1);
nT = length(t);

x = reshape(w(1 + (1:nT*nX)),nX,nT);
u = reshape(w((1 + nT*nX + 1):end),nU,nT);

utraj = PPTrajectory(foh(t,u));
utraj = setOutputFrame(utraj,sys.getInputFrame);

for i=1:nT
  xdot(:,i) = sys.dynamics(t(i),x(:,i),u(:,i));
end
xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
xtraj = setOutputFrame(xtraj,sys.getStateFrame);

end

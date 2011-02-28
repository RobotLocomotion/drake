function [w0,wlow,whigh,Flow,Fhigh,iGfun,jGvar,userfun,wrapupfun,iname,oname] = dircol_snopt_transcription(sys,costFun,finalCostFun,x0,utraj0,con,options)

% Direct collocation method.
%   Basic algorithm:  
%     u(t) is represented as a first-order spline
%     xc(t) is represented as a cubic spline
%     xd(t) is represented as a zero-order spline (zoh)
%   At every knot point, add a constraint enforcing the derivatives and
%   updates to match the spline derivatives, etc.
%
%  I use the algorithm as described in Enright91 and Hargraves86

%  todo: handle full set of constraints / options
%  todo: cost function should be on the outputs?  or just on the states?


nU = sys.getNumInputs();
nX = sys.getNumContStates();
nXd = sys.getNumDiscStates();
if (nXd>0)
  error('not implemented yet (but should be very straightforward)');
end

ts = sys.getSampleTime();
if (size(ts,2)>1 || any(ts~=0))
  error('not implemented yet (but should be very straightforward)');
end

t = utraj0.getBreaks();
tscale = 1;  % stretch time by this amount
u = utraj0.eval(t);
x = .1*randn(length(x0),length(t));   x(:,1) = x0;
nT = length(t);

%% Set up snopt inputs
w0 = [tscale; x(:); u(:)]';
whigh = [ tscale, repmat(inf,1,prod(size(x))), repmat(inf,1,prod(size(u)))];
wlow = [ tscale, repmat(-inf,1,prod(size(x))), repmat(-inf,1,prod(size(u)))];

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
    case 'x0'
      for f2 = fieldnames(con.x0)'
        switch(f2{1})
          case 'lb'
            wlow(1 + (1:nX)) = con.x0.lb;
          case 'ub'
            whigh(1 + (1:nX)) = con.x0.ub;
          otherwise 
            conwarn(f1,f2);
        end
      end
    case 'xf'
      xfind = 1 + nX*(nT-1) + (1:nX);
      for f2 = fieldnames(con.xf)'
        switch(f2{1})
          case 'lb'
            wlow(xfind) = con.xf.lb;
          case 'ub'
            whigh(xfind) = con.xf.ub;
          otherwise 
            conwarn(f1,f2);
        end
      end
      if all(wlow(xfind)==whigh(xfind))  % final value constraint
        for i=1:nX
          x(i,:) = linspace(x0(i),wlow(xfind(i)),nT);
        end
        w0(1+(1:(nT*nX))) = x(:);
      end
    case 'u'
      for f2 = fieldnames(con.u)'
        switch(f2{1})
          case 'lb'
            wlow(1 + nX*nT + 1 : end) = reshape(repmat(con.u.lb,1,nT),1,[]);
          case 'ub'
            whigh(1 + nX*nT + 1 : end) = reshape(repmat(con.u.ub,1,nT),1,[]);
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
  
  [nf,iGfun,jGvar,Fhigh,Flow,oname] = userfun_grad_ind(nT,nX,nU,options);
else
  [nf,iGfun,jGvar,Fhigh,Flow] = userfun_grad_ind(length(t),nX,nU,options);
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
  ucol = .5*(u(1:end-1)+u(2:end));  

  %% todo: vectorize this when possible

  % preallocate vars
  g = zeros(size(dt));
  dg = zeros(1,1+nX+nU,nT-1);
  xdot = zeros(nX,nT);
  dxdot = zeros(nX,1+nX+nU,nT);
  d = zeros(nX,nT-1);
  dd = zeros(nX,1+2*nX+2*nU,nT-1);
  
  % iterate through time
  xdot(:,1) = sys.dynamics(t(1),x(:,1),u(:,1));  
  df = sys.dynamicsGradients(t(1),x(:,1),u(:,1),1);
  dxdot(:,:,1) = df{1};  dxdot(:,1,1) = dxdot(:,1,1)*dtdw1(1); % d/d[tscale; x(:,1); u(:,1)]
  for i=1:(nT-1)  % compute dynamics and cost
    [g(i),dgc] = costFun(t(i),x(:,i),u(:,i));  dg(1,:,i) = dgc{1}; dg(1,1,i)=dg(1,1,i)*dtdw1(i);  % d/d[tscale; x(:,i); u(:,i)]
    dg(1,:,i) = [g(i)*ddtdw1(i),zeros(1,nX+nU)]+dt(i)*dg(1,:,i);  g(i) = g(i)*dt(i);  
    xdot(:,i+1) = sys.dynamics(t(i+1),x(:,i+1),u(:,i+1));
    df = sys.dynamicsGradients(t(i+1),x(:,i+1),u(:,i+1),1);
    dxdot(:,:,i+1) = df{1};  dxdot(:,1,1)=dxdot(:,1,1)*dtdw1(i+1); % d/d[tscale; x(:,i+1); u(:,i+1)]
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
    d(:,i)= sys.dynamics(tcol(i),xcol,ucol(:,i)) - xdotcol;
    df = sys.dynamicsGradients(tcol(i),xcol,ucol(:,i),1);
    dd(:,:,i) = df{1}*[dtcoldw1(i),zeros(1,2*nX+2*nU); dxcol; zeros(nU,1+nX), .5*eye(nU), zeros(nU,nX), .5*eye(nU)] - dxdotcol;  % d/d[tscale;x(:,i);u(:,i);x(:,i+1);u(:,i+1)]

    % for debugging
%    d(:,i)=xdotcol;
%    dd(:,:,i)=dxdotcol;
  end
  [h,dh] = finalCostFun(t(end),x(:,end)); dh = dh{1}; dh(1)=dh(1)*dtdw1(end);
  
  J = h + sum(g);
  dJ = [dh(1)+sum(dg(1,1,:)), reshape(dg(1,1+(1:nX),:),1,[]), dh(1,2:end), reshape(dg(1,1+nX+(1:nU),:),1,[]), zeros(1,nU)];
  
  f = [J; d(:)];
  G = [dJ(:); dd(:)];
end

function [nf, iGfun, jGvar, Fhigh, Flow, oname] = userfun_grad_ind(nT,nX,nU,options)
  % dJ:
  nf = 1;
  iGfun = repmat(1,1,1+nX*nT+nU*nT);  
  jGvar = (1:(1+nX*nT+nU*nT));
  oname{1} = 'J';
  
  Fhigh = inf;  Flow = -inf;
  
  % collocation constraints:
  for i=1:(nT-1)
    iGfun = [iGfun, nf+reshape(repmat(1:nX,(1+2*nX+2*nU),1)',1,[])];
    jGvar = [jGvar, reshape(repmat([1,1+nX*(i-1)+(1:nX),1+nX*nT+nU*(i-1)+(1:nU),1+nX*i+(1:nX),1+nX*nT+nU*i+(1:nU)],nX,1),1,[])];
    nf = nf+nX;
    if (nargout>5)
      for j=1:nX, oname= {oname{:},['d_',num2str(j),'(',num2str(i),')']}; end
    end
  end    
  
  Fhigh = [Fhigh,zeros(1,nf-1)];
  Flow = [Flow,zeros(1,nf-1)];
end


function [utraj,xtraj] = dircol_wrapup(sys,w,t,nX,nU)

t = t*w(1);
nT = length(t);

x = reshape(w(1 + (1:nT*nX)),nX,nT);
u = reshape(w((1 + nT*nX + 1):end),nU,nT);

utraj = PPTrajectory(foh(t,u));
for i=1:nT
  xdot(:,i) = sys.dynamics(t(i),x(:,i),u(:,i));
end
xtraj = PPTrajectory(pchipDeriv(t,x,xdot));

end
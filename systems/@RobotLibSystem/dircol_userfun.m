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
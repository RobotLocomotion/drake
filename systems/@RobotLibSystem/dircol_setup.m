function [w0,wlow,whigh,Flow,Fhigh,iGfun,jGvar,userfun,wrapupfun,iname,oname] = dircol_setup(sys,costFun,finalCostFun,x0,utraj0,con,options)
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
  % todo: handle the rest of the constraints or alarm for unsupported constraints
  if isfield(con,'x0')
    if isfield(con.x0,'lb'), wlow(1 + (1:nX)) = con.x0.lb; end
    if isfield(con.x0,'ub'), whigh(1 + (1:nX)) = con.x0.ub; end
  end
  if isfield(con,'xf')
    xfind = 1 + nX*(nT-1) + (1:nX);
    if isfield(con.xf,'lb'), wlow(xfind) = con.xf.lb; end
    if isfield(con.xf,'ub'), whigh(xfind) = con.xf.ub; end
    if all(wlow(xfind)==whigh(xfind))  % final value constraint
      for i=1:nX
        x(i,:) = linspace(x0(i),wlow(xfind(i)),nT);
      end
      w0(1+(1:(nT*nX))) = x(:);
    end
  end
  if isfield(con,'u')
    if isfield(con.u,'lb'), wlow(1 + nX*nT + 1 : end) = reshape(repmat(con.u.lb,1,nT),1,[]); end
    if isfield(con.u,'ub'), whigh(1 + nX*nT + 1 : end) = reshape(repmat(con.u.ub,1,nT),1,[]); end
  end
  if isfield(con,'T')
    if isfield(con.T,'lb'), wlow(1) = con.T.lb/(t(end)-t(1)); end
    if isfield(con.T,'ub'), whigh(1) = con.T.ub/(t(end)-t(1)); end
  end
  
  userfun = @(w) dircol_userfun(sys,w,costFun,finalCostFun,t,nX,nU,con,options);
  wrapupfun = @(w) dircol_wrapup(sys,w,t,nX,nU);
  
  if (options.grad_test)
    % print out the debugging key:
    iname = {'tscale'};
    for i=1:nT, for j=1:nX, iname= {iname{:},['x',num2str(j),'(',num2str(i),')']}; end, end
    for i=1:nT, for j=1:nU, iname= {iname{:},['u',num2str(j),'(',num2str(i),')']}; end, end

    [nf,iGfun,jGvar,Fhigh,Flow,oname] = userfun_grad_ind(nT,nX,nU,options);
  else
    [nf,iGfun,jGvar,Fhigh,Flow] = userfun_grad_ind(length(t),nX,nU,options);
    iname={};
    oname={};
  end
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
      for j=1:nX, oname= {oname{:},['d',num2str(j),'(',num2str(i),')']}; end
    end
  end    
  
  Fhigh = [Fhigh,zeros(1,nf-1)];
  Flow = [Flow,zeros(1,nf-1)];
end
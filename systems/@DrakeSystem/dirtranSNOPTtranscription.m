function [utraj,xtraj,info] = dirtranSNOPTtranscription(sys,costFun,finalCostFun,x0,utraj0,con,options)

% Open-loop trajectory optimization using the direct transcription method via
% SNOPT.  Roughly following the implementation described in Betts01, section 4.5.
%
% Inputs are:
%    sys                         - a class which implements the DrakeSystem interface
%    [g,dg] = costFun(t,x,u)     - the cost function, including gradients
%    [h,dh] = finalCostFun(t,x)  - the final time cost, including gradients
%    x0                          - initial conditions
%    utape0                      - an initial guess for the open-loop control
%    con                         - constraint structure
%    options                     - options struct populated by the fields below
%    

error('need to update this interface to be a transcription, instead of a complete trajectoryOptimization method'); 

checkDependency('snopt');

if (nargin<6) options = struct(); end
breaks = utraj0.getBreaks();
if (~isa(utraj0,'PPTrajectory') ||  (utraj0.pp.order>1) || (std(diff(breaks))>eps))
  warning('utraj0 will be treated as if it was a zero-order hold of the values at evenly spaced knot points'); 
  % note: i could consider updating the implementation to do better than this
end
tspan = linspace(breaks(1),breaks(end),length(breaks));
if (abs(tspan(1))>eps) warning('tspan starts at non-zero time, but the current code optimizes a trajectory from 0 to tspan(end)-tspan(1).  if your system has real time dependences, then you can complain about this!'); end
utape0 = utraj0.eval(tspan);
xtape0 = [];
[nU,nT] = size(utape0);
nX = length(x0);


%% implement options
if (~isfield(options,'transcription')) options.transcription = 'runge-kutta'; end
if (~isfield(options,'Tmin')) options.Tmin = 0;  end
if (~isfield(options,'Tmax')) options.Tmax = inf; end
if (~isfield(options,'Xmin')) options.Xmin = repmat(-inf,nX,1); end
if (~isfield(options,'Xmax')) options.Xmax = repmat(inf,nX,1); end
if (length(plant.umin)~=nU) error('umin is the wrong size'); end
if (length(plant.umax)~=nU) error('umax is the wrong size'); end
if (~isfield(options,'Umin')) options.Umin = plant.umin; end
if (~isfield(options,'Umax')) options.Umax = plant.umax;  end
if (~isfield(options,'bWarning')) options.bWarning = true; end
if (~isfield(options,'bGradTest')) options.bGradTest = false; end
if (~isfield(options,'user_constraint_fun')) options.user_constraint_fun = {}; end
if (~iscell(options.user_constraint_fun)) options.user_constraint_fun = {options.user_constraint_fun}; end
if (isfield(options,'snopt_options')) error('snopt options handling not implemented yet'); end
if (~isfield(options,'bx0free')) options.bx0free = false; end
if (~isfield(options,'xf')) options.xf = []; 
else typecheck(options.xf,{'double','Trajectory','TrajectoryLibrary'}); end 
if (~isfield(options,'maxDT')) options.maxDT = 0.1; end

if (~isfield(options,'xtraj0')) 
  if (~isempty(options.xf) && isa(options.xf,'double'))
    for i=1:nX, xtape0(i,:) = linspace(x0(i),options.xf(i),nT); end
  else
    options.xtraj0 = plant.simulate(OpenloopControl(utraj0),tspan,x0);
  end
end
if (isempty(xtape0))
  xtape0 = options.xtraj0.eval(tspan);
  if any(abs(xtape0(:,1)-x0)>eps) error('xtraj(tspan(1)) doesn''t match x0'); end
end

%% basic error checking. (should do a lot more)
if (exist('snopt')==0) 
  error('dirtran needs the matlab wrapper for SNOPT to be in the matlab path.  if you don''t have it, you can download a student version here: http://www.scicomp.ucsd.edu/~peg/Software.html');
end

if (options.bx0free)
  error('not implemented yet.');
end

%% Set up snopt inputs
w0 = [tspan(end); xtape0(:); utape0(:) ];  
xhigh = [options.Tmax; repmat(options.Xmax,nT,1); repmat(options.Umax,nT,1)];
xlow = [options.Tmin; repmat(options.Xmin,nT,1); repmat(options.Umin,nT,1)];
N = length(w0);

global SNOPT_USERFUN;
SNOPT_USERFUN = @(w) userfun(w,plant,COSTFUN,FINALCOSTFUN,x0,nT,nX,nU,options);

[nf,iGfun,jGvar,Fhigh,Flow] = userfun_grad_ind(nT,nX,nU,options);

%% very useful for debugging:
if (options.bGradTest)
  % print out the debugging key:
  iname = {'tf'};
  for i=1:nT, for j=1:nX, iname= {iname{:},['x',num2str(j),'(',num2str(i),')']}; end, end
  for i=1:nT, for j=1:nU, iname= {iname{:},['u',num2str(j),'(',num2str(i),')']}; end, end

  [nf,iGfun,jGvar,Fhigh,Flow,oname] = userfun_grad_ind(nT,nX,nU,options);

  gradTest(@(w)gradTestFun(w,iGfun,jGvar),w0,struct('input_name',{{iname}},'output_name',{oname},'tol',.01));
end
  
%% Run SNOPT
[w,F,info] = snopt(w0,xlow,xhigh,Flow,Fhigh,'snopt_userfun',0,1,[],[],[],iGfun,jGvar);

if (info~=1 && options.bWarning) 
  warning(['SNOPT exited w/ info = ',num2str(info),'.  Check p19 of Gill06 for more information.']);  
end

% sometimes useful to look here, too:
%snopt_gradtest(w,'snopt_userfun',iGfun,jGvar);  return;

  tf = w(1);
  tspan=linspace(0,tf,nT);  
  xtape = reshape(w(1 + (1:nX*nT)),nX,nT);
  utape = reshape(w(1+nX*nT+(1:nU*nT)),nU,nT);

  for i=1:nT
    xdot(:,i) = plant.dynamics(tspan(i),xtape(:,i),utape(:,i));
  end
  xtraj = PPTrajectory(piecewiseCubicPoly(tspan,xtape,xdot(:,1:(end-1)),xdot(:,2:end)));
  xtraj = setOutputFrame(xtraj,sys.getStateFrame);
  utraj = PPTrajectory(zoh(tspan,utape));
  utraj = setOutputFrame(utraj,sys.getInputFrame);

  
  if (info==1 && tf/N > options.maxDT)  % then add more elements to the tape and call dircol again
    if (options.bWarning)
      warning('dt > maxDT, upsampling and rerunning dircol');
    end
    tspan = reshape([tspan;tspan+diff(tspan)/2],1,[]); 
    utape0 = reshape(repmat(utape,2,1),nU,[]);
    utraj = PPTrajectory(zoh(tspan(1:end-1),utape0(:,1:end-1)));
    options.xtraj = xtraj;
    [xtraj,utraj,info] = dirtran(plant,COSTFUN,FINALCOSTFUN,xtraj.eval(tspan(1)),utraj,options);
  end
  
end


function [f,G] = userfun(w,plant,COSTFUN,FINALCOSTFUN,x0,nT,nX,nU,options)
  tf = w(1);
  xtape = reshape(w(1 + (1:nX*nT)),nX,nT);
  utape = reshape(w(1+nX*nT+(1:nU*nT)),nU,nT);
  dt = tf/(nT-1);
  ddtdT = 1/(nT-1);

%  figure(1); h=plot(xtape(1,:),xtape(2,:)); drawnow;  delete(h);
  
  % cost function
  f=0; G=zeros(size(w));
  for i=1:nT
    t=dt*(i-1);
    dtdT = ddtdT*(i-1);
    
    [g,dg] = geval(COSTFUN,t,xtape(:,i),utape(:,i));
    f = f+dt*g;
    G(1) = G(1) + ddtdT*g;
    dg{1}(1) = dg{1}(1)*dtdT;
    ind = [1,1+(i-1)*nX+(1:nX),1+nX*nT+(i-1)*nU+(1:nU)];
    G(ind) = G(ind)+dt*dg{1}';  % dJ/d[T;xi;ui]
  end    
  [h,dh] = geval(FINALCOSTFUN,tf,xtape(:,end));
  f = f+h;
  ind = [1,1+(nT-1)*nX+(1:nX)];
  G(ind) = G(ind)+dh{1}';
  
  if (~options.bx0free)
    f = [f; xtape(:,1)-x0];
    G = [G; ones(nX,1)];
  end
  
  if (~isempty(options.xf))
    if (isa(options.xf,'double'))
      f = [f; xtape(:,end)-options.xf];
      G = [G; reshape(eye(nX),nX*nX,1)];
    elseif (isa(options.xf,'Trajectory')||isa(options.xf,'TrajectoryLibrary'))
      [v,dvdx] = options.xf.vector_to(xtape(:,end));
      f = [f; v];
      G = [G; reshape(dvdx,nX*nX,1)];
    end
  end
  
  % constraints (make sure that xtape is a trajectory of the system)
  switch (options.transcription)
    case 'euler'
      for i=1:(nT-1)
        t=dt*(i-1);

        [xdot,dxdot] = geval(@plant.dynamics,t,xtape(:,i),utape(:,i));
        f = [f; xtape(:,i+1) - xtape(:,i) - dt*xdot];
        % df = df/d[T;xi;ui]
        df = -dt*dxdot;  
        df(:,1) = df(:,1) - ddtdT*xdot;
        df(:,1+(1:nX)) = df(:,1+(1:nX)) - eye(nX);

        G = [G;  ...
          reshape(df,nX*(1+nX+nU),1);                 % df/d[T;xi;ui]
          ones(nX,1)];                                % diag(df(j)dx(j,i+1))
      end

    case 'runge-kutta'
      % going to need one extra control for this
      utape(:,end+1) = utape(:,end);

      for i=1:(nT-1)
        t=dt*(i-1);
        dtdT = ddtdT*(i-1);
        % NOTE: this code currently assumes a zero-order hold on actions
        
        [xdot,df] = geval(@plant.dynamics,t,xtape(:,i),utape(:,i));
        k1 = dt*xdot;
        % dk1 = dk1/d[T;xi;ui]
        dk1 = dt*df;  
        dk1(:,1) = dk1(:,1) + ddtdT*xdot;

        [xdot,df] = geval(@plant.dynamics,t+dt/2,xtape(:,i)+k1/2,utape(:,i));
        k2 = dt*xdot;
        dk2 = dt*df*[dtdT + ddtdT/2,zeros(1,nX+nU); dk1/2+[zeros(nX,1),eye(nX),zeros(nX,nU)]; zeros(nU,1+nX), eye(nU)];
        dk2(:,1) = dk2(:,1) + ddtdT*xdot;
        
        [xdot,df] = geval(@plant.dynamics,t+dt/2,xtape(:,i)+k2/2,utape(:,i));
        k3 = dt*xdot;
        dk3 = dt*df*[dtdT + ddtdT/2,zeros(1,nX+nU); dk2/2+[zeros(nX,1),eye(nX),zeros(nX,nU)];  zeros(nU,1+nX), eye(nU)];  
        dk3(:,1) = dk3(:,1) + ddtdT*xdot;

        [xdot,df] = geval(@plant.dynamics,t+dt,xtape(:,i)+k3,utape(:,i+1));
        k4 = dt*xdot;
        dk4 = dt*df*[dtdT + ddtdT,zeros(1,nX+nU); dk3+[zeros(nX,1),eye(nX),zeros(nX,nU)];  zeros(nU,1+nX), zeros(nU)];  
        dk4(:,1) = dk4(:,1) + ddtdT*xdot;

        f = [f; xtape(:,i+1) - xtape(:,i) - 1/6*(k1+2*k2+2*k3+k4)];
        % collect df/d[T;xi;ui]
        dnf = -[zeros(nX,1),eye(nX),zeros(nX,nU)]-1/6*(dk1 + 2*dk2 + 2*dk3 + dk4);
        
        G = [G;  ...
          reshape(dnf,nX*(1+nX+nU),1); ...                % df/d[T;xi;ui]
          reshape(-1/6*dt*df{1}(:,(end-nU+1):end),nX*nU,1); ...  % df/du(:,i+1)
          ones(nX,1)];                                 % df(j)/dx(j,i+1)
      end

      
    otherwise
      error(['transcription ',options.transcription,' not implemented yet']);
  end
  
  for i=1:length(options.user_constraint_fun)
   [uf,uG]=options.user_constraint_fun{i}(nT,nX,nU,w);
   f = [f;uf];
   G = [G; uG];
  end
end
  
function [nf, iGfun, jGvar, Fhigh, Flow, oname] = userfun_grad_ind(nT,nX,nU,options)

  N = 1 + nX*nT + nU*nT;
  
  iGfun = ones(N,1);        % dJ/d[T;x;u]
  jGvar = [1:N]';      % dJ/d[T,X,U]

  nf = 1;
  if (nargout>5) oname = {'J'}; end
  
  if (~options.bx0free)
    iGfun = [iGfun; nf+(1:nX)'];
    jGvar = [jGvar; 1+(1:nX)'];
    nf = nf+nX;
    if (nargout>5) 
      for i=1:nX, oname= {oname{:},['x0_',num2str(i)]}; end
    end
  end
  if (~isempty(options.xf))
    iGfun = [iGfun; repmat(nf+(1:nX)',nX,1)]; 
    jGvar = [jGvar; 1+(nT-1)*nX+reshape(repmat(1:nX,nX,1),nX*nX,1)];
    nf = nf+nX;
    if (nargout>5) 
      for i=1:nX, oname= {oname{:},['xf_',num2str(i)]}; end
    end
  end

  switch (options.transcription)
    case 'euler'
      for i=1:(nT-1)
        iGfun = [iGfun; ...
          nf + repmat(1:nX,1,1+nX+nU+1)' ];

        jGvar = [jGvar; ...
          repmat(1,nX,1); ...   % df/dT
          1 + (i-1)*nX + reshape(repmat(1:nX,nX,1),nX*nX,1); ...  % df/dxi
          1 + nX*nT + (i-1)*nU + reshape(repmat(1:nU,nX,1),nU*nX,1); ...          % df/dui
          1 + i*nX + (1:nX)' ];                               % df(j)/dx(j,i+1)

        nf = nf + nX;
        if (nargout>5)
          for j=1:nX, oname= {oname{:},['x',num2str(j),'(',num2str(i+1),')']}; end
        end
      end

    case 'runge-kutta'
      for i=1:(nT-1)
        iGfun = [iGfun; ...
          nf + repmat(1:nX,1,1+nX+2*nU+1)' ];

        jGvar = [jGvar; ...
          repmat(1,nX,1);  ...     % df/dT
          1 + (i-1)*nX + reshape(repmat(1:nX,nX,1),nX*nX,1); ...  % df/dxi
          1 + nX*nT + (i-1)*nU + reshape(repmat(1:nU,nX,1),nU*nX,1); ...          % df/dui
          1 + nX*nT + i*nU + reshape(repmat(1:nU,nX,1),nU*nX,1); ...     % df/du(i+1)
          1 + i*nX + (1:nX)' ];                                   % df(j)/dx(j,i+1)

        nf = nf + nX;
        if (nargout>5)
          for j=1:nX, oname= {oname{:},['x',num2str(j),'(',num2str(i+1),')']}; end
        end
      end
      
    otherwise
      error(['transcription ',options.transcription,' not implemented yet']);
  end

  Fhigh = [inf;zeros(nf-1,1)];
  Flow = [-inf;zeros(nf-1,1)];
  
  if length(options.user_constraint_fun)>0
    warning('i updated the arrangement of the alpha vector.  you''ll need to update user constraint funs. sorry..');
  end
  
  for i=1:length(options.user_constraint_fun)
    [unf,uiGfun,ujGvar,uFhigh,uFlow]=options.user_constraint_fun{i}(nT,nX,nU,[]);
    iGfun = [iGfun; uiGfun+nf];
    jGvar = [jGvar; ujGvar];
    Fhigh = [Fhigh; uFhigh];
    Flow = [Flow; uFlow];
    nf = nf + unf;
    if (nargout>5)
      for j=1:unf, oname= {oname{:},['user',num2str(j)]}; end
    end
  end
end


function [f,df] = gradTestFun(w,iGfun,jGvar)
  [f,G] = snopt_userfun(w);
  df = sparse(iGfun,jGvar,G);
end



function [w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,userfun,wrapupfun,iname,oname] = dircolSNOPTtranscription(sys,costFun,finalCostFun,x0,utraj0,con,options)

  hybrid_options=options;  % keep a backup
  if (~isfield(con,'mode')) error('con.mode must be defined for HybridDrakeSystems'); end
  for i=1:length(x0)
    if (length(x0{i})~=sys.modes{con.mode{i}.mode_num}.getNumContStates()), error('x0 should NOT have the mode variable as the first element.'); end
  end
  for m=1:length(con.mode)
    % todo: check here that utraj0{m} starts at t=0?
    if (isfield(hybrid_options,'trajectory_cost_fun'))
      options.trajectory_cost_fun = hybrid_options.trajectory_cost_fun{m};
    end
    [w0{m},wlow{m},whigh{m},Flow{m},Fhigh{m},A{m},iAfun{m},jAvar{m},iGfun{m},jGvar{m},mode_userfun{m},mode_wrapup{m},mode_iname{m},mode_oname{m}] = dircolSNOPTtranscription(sys.modes{con.mode{m}.mode_num},costFun{m},finalCostFun{m},x0{m},utraj0{m},rmfield(con.mode{m},'mode_num'),options);
    tOrig{m} = utraj0{m}.getBreaks();
    N(m) = length(w0{m});
    nT(m) = length(tOrig{m});
    nf(m) = length(Flow{m});
    if (m>1)
      nf(m)=nf(m)-1;  % don't duplicate cost function
      Flow{1}(1)=Flow{1}(1)+Flow{m}(1); Flow{m} = Flow{m}(2:end);
      Fhigh{1}(1)=Fhigh{1}(1)+Fhigh{m}(1); Fhigh{m} = Fhigh{m}(2:end);
      njind = (iAfun{m}~=1); 
      iAfun{m}(njind) = iAfun{m}(njind)+sum(nf(1:m-1))-1;  %subtract 1 to remove J ind
      jAvar{m} = jAvar{m}+sum(N(1:m-1));
      njind = (iGfun{m}~=1);  
      iGfun{m}(njind) = iGfun{m}(njind)+sum(nf(1:m-1))-1;  %subtract 1 to remove J ind
      jGvar{m} = jGvar{m}+sum(N(1:m-1));
      mode_oname{m}=mode_oname{m}(2:end);
    end
  end
  m=m+1;
  [nf(m), A{m} ,iAfun{m} ,jAvar{m}, iGfun{m}, jGvar{m}, Fhigh{m}, Flow{m}, fsm_oname] = fsmObjFun_ind(sys,N,nT,con,options);
  iAfun{m} = iAfun{m}+sum(nf(1:m-1));
  iGfun{m} = iGfun{m}+sum(nf(1:m-1));
  w0=cell2mat(w0'); wlow=cell2mat(wlow'); whigh=cell2mat(whigh'); Flow=cell2mat(Flow'); Fhigh=cell2mat(Fhigh'); A=cell2mat(A'); iAfun=cell2mat(iAfun'); jAvar=cell2mat(jAvar'); iGfun=cell2mat(iGfun'); jGvar=cell2mat(jGvar');

  % handle additional constraints
  for f=fieldnames(con)'
    switch(f{1})
      case 'mode'
        continue  %handled above
      case 'periodic'
        continue  %handled in fsmObjFun_ind
      case 'u_const_across_transitions'
        continue  %handled in fsmObjFun_ind
      otherwise
        warning([f{1},' constraint not handled by hybrid dircol (at least not yet)']);
    end
  end
  
  userfun = @(w) dircol_userfun(sys,w,mode_userfun,tOrig,N,con,options);
  wrapupfun = @(w) dircol_wrapup(sys,w,mode_wrapup,tOrig,N,con,options);
  
  if (options.grad_test)
    iname = {};  oname = {};
    for i=1:length(con.mode), 
      for j=1:length(mode_iname{i}), 
        iname = {iname{:},['mode ',num2str(i),', ',mode_iname{i}{j}]};
      end
      for j=1:length(mode_oname{i}),
        oname = {oname{:},['mode ',num2str(i),', ',mode_oname{i}{j}]};
      end
    end
    oname = {oname{:},fsm_oname{:}};
  else
    iname={};
    oname={};
  end

  
  
end

function [f,G] = dircol_userfun(sys,w,userfun,tOrig,N,con,options)

  ind = 0;
  f=0;
  G=[];
  for m=1:length(N)
    [cf,cG] = userfun{m}(w(ind+(1:N(m))));
    ind = ind+N(m);
    f(1) = f(1)+cf(1);  % add costs
    f = [f; cf(2:end)]; % tally up constraints
    G = [G; cG];
  end

%   figure(1); clf; hold on;
  
  nX = sys.getNumContStates();
  nU = sys.getNumInputs();
  %% additional fsm constraints:
  for m=1:length(N)-1
    from_mode = con.mode{m}.mode_num; 
    to_mode = con.mode{m+1}.mode_num;
    nXm=sys.modes{from_mode}.getNumContStates();%This is the number of states in the prior mode,m for "minus"
    nUm=sys.modes{from_mode}.getNumInputs();%Number of inputs in the prior mode
    nXp=sys.modes{to_mode}.getNumContStates();%This is the number of states in the posterior mode, p for "plus"
    nUp=sys.modes{to_mode}.getNumInputs();%This is the number of inputs in the posterior mode.
    if (m>1) from_ind = sum(N(1:m-1)); else from_ind = 0; end
    nT = length(tOrig{m}); 

    % plot trajectory (this is just a debugging hack for the compass gait):
%     x=reshape(w(from_ind+1+(1:nX*nT)),nX,nT); 
%     plot(x(1,:),x(3,:),'r');
%     plot(x(2,:),x(4,:),'b');
    
    % final value for each mode (except the last) needs to have or(relevant zcs=0).  not
    % just any zc, but a zc that transitions me to the correct next node.
    tscale = w(from_ind+1);
    tc = tscale*tOrig{m}(end);
    xc=w(from_ind+1+(nT-1)*nXm+(1:nXm));uc=w(from_ind+1+nT*nXm+(nT-1)*nUm+(1:nUm));
    zc = 1; dzc = zeros(1,1+nXm+nUm);  % d/d[tscale,xc,uc]
    min_g = inf; min_g_ind = 0;
    for i=find(sys.target_mode{from_mode}==to_mode | sys.target_mode{from_mode}==0)
      [g,dg] = geval(sys.guard{from_mode}{i},sys,tc,xc,uc,options);
      dg(1) = dg(1)*tOrig{m}(end);
      if (g<min_g), min_g = g; min_g_ind = i; end 
      dzc = dzc*g + zc*dg;
      zc = zc*g;  % multiply zcs to implement logical OR.
    end
    f = [f; zc];
    G = [G; dzc(:)];
    
%    continue;
  
    % initial value for each mode (except the first) needs to equal final
    % value of the previous after the transition update.
    if (min_g_ind<1) error('no applicable zero crossings defined'); end
    to_ind = from_ind+N(m);
    to_x0 =  w(to_ind+1+(1:nXp));
    [to_x,to_mode,status,dto_x] = geval(3,sys.transition{from_mode}{min_g_ind},sys,from_mode,tc,xc,uc,options);
    dto_x(:,1) = []; 
    dto_x(:,1) = dto_x(:,1)*tOrig{m}(end);; % zap grad with respect to mode
    f = [f; to_x - to_x0];
    G = [G; dto_x(:); -ones(nXp,1)];  % df/d[tc,xc,uc]; df/d[to_x0]
  end
  
  % draw last mode
%   from_ind = sum(N(1:end-1));
%   x=reshape(w(from_ind + 1+(1:nX*nT)),nX,nT);
%   plot(x(1,:),x(3,:),'r');
%   plot(x(2,:),x(4,:),'b');
%   drawnow;


  if (isfield(con,'periodic') && con.periodic)
    f = [f;zeros(nX+nU,1)];  % implemented as linear constraint below, just put in zero here
  end
  if (isfield(con,'u_const_across_transitions') && con.u_const_across_transitions)
    f = [f;zeros((length(N)-1)*nU,1)];
  end
  
end


function [nf, A, iAfun, jAvar, iGfun, jGvar, Fhigh, Flow, oname] = fsmObjFun_ind(sys,N,nT,con,options)
  nX = sys.getNumContStates();
  nU = sys.getNumInputs();
  
 
  %% additional fsm constraints:
  nf = 0;
  iGfun=[]; jGvar=[];   oname={};
  A=[]; iAfun=[]; jAvar=[];
  
  % for debugging
%  Fhigh=[]; Flow=[]; return
  
  for m=1:length(N)-1
    from_mode = con.mode{m}.mode_num; 
    to_mode = con.mode{m+1}.mode_num;
    nXm=sys.modes{from_mode}.getNumContStates();%This is the number of states in the prior mode,m for "minus"
    nUm=sys.modes{from_mode}.getNumInputs();%Number of inputs in the prior mode
    nXp=sys.modes{to_mode}.getNumContStates();%This is the number of states in the posterior mode, p for "plus"
    nUp=sys.modes{to_mode}.getNumInputs();%This is the number of inputs in the posterior mode.
    % final value for each mode (except the last) needs to have zc=0.
    if (m>1) from_ind = sum(N(1:m-1)); else from_ind = 0; end
    iGfun = [iGfun; nf + ones(1+nXm+nUm,1)];
    jGvar = [jGvar; from_ind + [1,1+(nT(m)-1)*nXm+(1:nXm),1+nT(m)*nXm+(nT(m)-1)*nUm+(1:nUm)]'];  
    nf = nf + 1;
    
    % for debugging
%    oname = {oname{:}, ['mode(',num2str(m),').xf zc']};  continue;
    
    % initial value for each mode (except the first) needs to equal final
    % value of the previous after the transition update.
    to_ind = from_ind+N(m);
    iGfun = [iGfun; nf + reshape(repmat(1:nXp,1+nXm+nUm,1)',[],1); nf + (1:nXp)'];
    jGvar = [jGvar; from_ind + reshape(repmat([1,1+(nT(m)-1)*nXm+(1:nXm),1+nT(m)*nXm+(nT(m)-1)*nUm+(1:nUm)],nXp,1),[],1); to_ind+1+(1:nXp)'];
    nf = nf + nXp;

    if (options.grad_test) 
      oname = {oname{:}, ['mode ',num2str(m),', x(tf) zc']};
      for j=1:nX, 
        oname = {oname{:}, ['transition(mode ',num2str(m),', x_',num2str(j),'(tf)) - mode ',num2str(m+1),', x_',num2str(j),'(t0)']}; 
      end
    end
  end
  Fhigh = zeros(nf,1);
  Flow = Fhigh;

  if (isfield(con,'periodic') && con.periodic)
    % then add linear constraints  x0[i]=xf[i] and u0[i]=uf[i]
    A = [A; repmat([1;-1],nX+nU,1)];
    iAfun = [iAfun; nf+reshape(repmat(1:(nX+nU),2,1),[],1)];
    x0ind = 1+(1:nX); xfind = sum(N)-nU*nT(end)-nX + (1:nX);
    u0ind = 1+nX*nT(1)+(1:nU); ufind = sum(N)-nU + (1:nU);   
    jAvar = [jAvar; reshape([x0ind,u0ind; xfind,ufind],[],1)];
    Fhigh = [Fhigh; zeros(nX+nU,1)];
    Flow = [Flow; zeros(nX+nU,1)];
    if (nargout>5)
      for j=1:nX, oname= {oname{:},['con.periodic_x',num2str(j)]}; end
      for j=1:nU, oname= {oname{:},['con.periodic_u',num2str(j)]}; end
    end      
    nf = nf + nX+nU;
  end
  
  if (isfield(con,'u_const_across_transitions') && con.u_const_across_transitions)
    for i=1:length(N)-1
        from_mode = con.mode{i}.mode_num; 
        to_mode = con.mode{i+1}.mode_num;
        nXm=sys.modes{from_mode}.getNumContStates();%This is the number of states in the prior mode,m for "minus"
        nUm=sys.modes{from_mode}.getNumInputs();%Number of inputs in the prior mode
        nXp=sys.modes{to_mode}.getNumContStates();%This is the number of states in the posterior mode, p for "plus"
        nUp=sys.modes{to_mode}.getNumInputs();%This is the number of inputs in the posterior mode.
      A = [A; repmat([1;-1],nUm,1)];
      iAfun = [iAfun;nf+reshape(repmat(1:nUm,2,1),[],1)];
      u0ind = sum(N(1:i))+1+nXp*nT(i+1)+(1:nUp);
      ufind = sum(N(1:(i-1)))+1+nXm*nT(i)+nUm*(nT(i)-1)+(1:nUm);
      jAvar = [jAvar; reshape([u0ind'; ufind'],[],1)];
      Fhigh = [Fhigh; zeros(nUm,1)];
      Flow = [Flow; zeros(nUm,1)];
      if (nargout>5)
        for j=1:nUm, oname= {oname{:},['(mode',num2str(i+1),'.uf - mode',num2str(i),'.u0)_',num2str(j)]}; end
      end
      nf = nf + nUm;
    end
  end

end

function [utraj,xtraj] = dircol_wrapup(sys,w,mode_wrapupfun,tOrig,N,con,options)

  t=0;
  ind=0;
  for m=1:length(N)
    [utraj{m},mode_xtraj] = mode_wrapupfun{m}(w(ind +(1:N(m))));
    utraj{m} = shiftTime(utraj{m},t);
    mode_xtraj = shiftTime(mode_xtraj,t);
    
    tnext=t+w(ind+1)*tOrig{m}(end);
    ind = ind+N(m);

    mtraj=PPTrajectory(zoh([t tnext],repmat(con.mode{m}.mode_num,1,2)));
    mode_numxc = sys.modes{con.mode{m}.mode_num}.getNumContStates();
    numxc = sys.getNumContStates();
    if (mode_numxc<numxc)
        zeropadding = PPTrajectory(zoh([t tnext],zeros(numxc-mode_numxc,2)));
        xtraj{m} = MixedTrajectory({mtraj,mode_xtraj,zeropadding},{1,1+[1:mode_numxc],1+[1+mode_numxc:numxc]});
    else
        xtraj{m} = MixedTrajectory({mtraj,mode_xtraj},{1,1+[1:sys.getNumContStates()]});
    end
    %xtraj{m} = MixedTrajectory({mtraj,mode_xtraj},{1,1+[1:sys.modes{con.mode{m}.mode_num}.getNumContStates()]});
    xtraj{m} = setOutputFrame(xtraj{m},sys.getStateFrame);
    t=tnext;
  end
  
  utraj = HybridTrajectory(utraj);
  utraj = setOutputFrame(utraj,sys.getInputFrame);
  xtraj = HybridTrajectory(xtraj);

end

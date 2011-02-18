function [w0,wlow,whigh,Flow,Fhigh,iGfun,jGvar,userfun,wrapupfun,iname,oname] = dircol_setup(sys,costFun,finalCostFun,x0,utraj0,con,options)

  if (~isfield(con,'mode')) error('con.mode must be defined for FiniteStateMachines'); end
  for i=1:length(x0)
    if (length(x0{i})~=sys.getNumContStates()), error('x0 should NOT have the mode variable as the first element.'); end
  end
  for m=1:length(con.mode)
    % todo: check here that utraj0{m} starts at t=0?
    [w0{m},wlow{m},whigh{m},Flow{m},Fhigh{m},iGfun{m},jGvar{m},mode_userfun{m},mode_wrapup{m},mode_iname{m},mode_oname{m}] = dircol_setup(sys.modes{con.mode(m).mode_num},costFun{m},finalCostFun{m},x0{m},utraj0{m},con.mode(m),options);
    tOrig{m} = utraj0{m}.getBreaks();
    N(m) = length(w0{m});
    nT(m) = length(tOrig{m});
    nf(m) = length(Flow{m});
    if (m>1)
      nf(m)=nf(m)-1;  % don't duplicate cost function
      Flow{1}(1)=Flow{1}(1)+Flow{m}(1); Flow{m} = Flow{m}(2:end);
      Fhigh{1}(1)=Fhigh{1}(1)+Fhigh{m}(1); Fhigh{m} = Fhigh{m}(2:end);
      njind = (iGfun{m}~=1);  
      iGfun{m}(njind) = iGfun{m}(njind)+sum(nf(1:m-1))-1;  %subtract 1 to remove J ind
      jGvar{m} = jGvar{m}+sum(N(1:m-1));
      mode_oname{m}=mode_oname{m}(2:end);
    end
  end
  m=m+1;
  [nf(m), iGfun{m}, jGvar{m}, Fhigh{m}, Flow{m}, fsm_oname] = fsmObjFun_ind(sys,N,nT,options);
  iGfun{m} = iGfun{m}+sum(nf(1:m-1));
  w0=[w0{:}]; wlow=[wlow{:}]; whigh=[whigh{:}]; Flow=[Flow{:}]; Fhigh=[Fhigh{:}]; iGfun=[iGfun{:}]; jGvar=[jGvar{:}];
  userfun = @(w) dircol_userfun(sys,w,mode_userfun,tOrig,N,con,options);
  wrapupfun = @(w) dircol_wrapup(sys,w,mode_wrapup,tOrig,N,con,options);;
  if (options.grad_test)
    iname = {};  oname = {};
    for i=1:length(con.mode), 
      for j=1:length(mode_iname{i}), 
        iname = {iname{:},['m',num2str(i),' ',mode_iname{i}{j}]};
      end
      for j=1:length(mode_oname{i}),
        oname = {oname{:},['m',num2str(i),' ',mode_oname{i}{j}]};
      end
    end
    oname = {oname{:},fsm_oname{:}};
  else
    iname={};
    oname={};
  end

end


function [nf, iGfun, jGvar, Fhigh, Flow, oname] = fsmObjFun_ind(sys,N,nT,options)
  nX = sys.getNumContStates();
  nU = sys.getNumInputs();
 
  %% additional fsm constraints:
  nf = 0;
  iGfun=[]; jGvar=[];   oname={};
  
  % for debugging
%  Fhigh=[]; Flow=[]; return
  
  for m=1:length(N)-1
    % final value for each mode (except the last) needs to have zc=0.
    if (m>1) from_ind = cumsum(N(1:m-1)); else from_ind = 0; end
    iGfun = [iGfun, nf + ones(1,1+nX+nU)];
    jGvar = [jGvar, from_ind + [1,1+(nT(m)-1)*nX+(1:nX),1+nT(m)*nX+(nT(m)-1)*nU+(1:nU)]];  
    nf = nf + 1;
    
    % for debugging
%    oname = {oname{:}, ['mode(',num2str(m),').xf zc']};  continue;
    
    % initial value for each mode (except the first) needs to equal final
    % value of the previous after the transition update.
    to_ind = from_ind+N(m);
    iGfun = [iGfun, nf + reshape(repmat(1:nX,1+nX+nU,1)',1,[]), nf + (1:nX)];
    jGvar = [jGvar, from_ind + reshape(repmat([1,1+(nT(m)-1)*nX+(1:nX),1+nT(m)*nX+(nT(m)-1)*nU+(1:nU)],nX,1),1,[]), to_ind+1+(1:nX)];
    nf = nf + nX;

    if (options.grad_test) 
      oname = {oname{:}, ['mode(',num2str(m),').xf zc']};
      for j=1:nX, 
        oname = {oname{:}, ['update(mode(',num2str(m),').xf)(i) - mode(',num2str(m+1),').x0']}; 
      end
    end
  end
  Fhigh = zeros(1,nf);
  Flow = Fhigh;
end
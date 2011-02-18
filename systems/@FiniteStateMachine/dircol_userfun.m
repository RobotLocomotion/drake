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
  
%  return;
  
  nX = sys.getNumContStates();
  nU = sys.getNumInputs();
  %% additional fsm constraints:
  for m=1:length(N)-1
    from_mode = con.mode(m).mode_num; 
    to_mode = con.mode(m+1).mode_num;
    
    % final value for each mode (except the last) needs to have or(relevant zcs=0).  not
    % just any zc, but a zc that transitions me to the correct next node.
    if (m>1) from_ind = cumsum(N(1:m-1)); else from_ind = 0; end
    tscale = w(from_ind+1);
    nT = length(tOrig{m}); 
    tc = tscale*tOrig{m}(end); xc = w(from_ind+1+(nT-1)*nX+(1:nX));  uc = w(from_ind+1+nT*nX+(nT-1)*nU+(1:nU));
    zc = 1; dzc = zeros(1,1+nX+nU);  % d/d[tscale,xc,uc]
    min_g = inf; min_g_ind = 0;
    for i=find(sys.target_mode{from_mode}==to_mode)
      [g,dg] = sys.guard{from_mode}{i}(sys,tc,xc,uc);
      dg = dg{1}; dg(1) = dg(1)*tOrig{m}(end);
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
    to_x0 =  w(to_ind+1+(1:nX));
    [to_x,status,dto_x] = sys.update{from_mode}{min_g_ind}(sys,tc,xc,uc);
    dto_x = dto_x{1};  dto_x(1,:) = dto_x(1,:)*tOrig{m}(end);
    f = [f; to_x - to_x0];
    G = [G; dto_x(:); -ones(nX,1)];  % df/d[tc,xc,uc]; df/d[to_x0]
  end
  
end


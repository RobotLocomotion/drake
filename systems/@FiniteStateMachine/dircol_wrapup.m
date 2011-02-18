function [utraj,xtraj] = dircol_wrapup(sys,w,mode_wrapupfun,tOrig,N,con,options)

  t=0;
  ind=0;
  for m=1:length(N)
    [utraj{m},mode_xtraj] = mode_wrapupfun{m}(w(ind +(1:N(m))));
    utraj{m} = shiftTime(utraj{m},t);
    mode_xtraj = shiftTime(mode_xtraj,t);
    
    tnext=t+w(ind+1)*tOrig{m}(end);
    ind = ind+N(m);

    mtraj=PPTrajectory(zoh([t tnext],repmat(con.mode(m).mode_num,1,2)));
    xtraj{m} = MixedTrajectory({mtraj,mode_xtraj},{1,1+[1:sys.getNumContStates()]});
    t=tnext;
  end
  
  utraj = HybridTrajectory(utraj);
  xtraj = HybridTrajectory(xtraj);

end

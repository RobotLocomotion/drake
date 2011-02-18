function traj = simulate(obj,tspan,x0,options)

typecheck(tspan,'double');
if (length(tspan)<2) error('length(tspan) must be > 1'); end
if (nargin>3) error('no options implemented yet'); end
mdl = getModel(obj);

if (strcmp(get_param(mdl,'SimulationStatus'),'paused'))
  feval(mdl,[],[],[],'term');  % terminate model, in case it was still running before
end

pstruct = obj.simulink_params;
pstruct.StartTime = num2str(tspan(1));
pstruct.StopTime = num2str(tspan(end));

if (nargin>2) % handle initial conditions
  x0 = obj.stateVectorToStructure(x0);
  assignin('base',[mdl,'_x0'],x0);
  pstruct.InitialState = [mdl,'_x0'];
  pstruct.LoadInitialState = 'on';
end

  function traj = makeSubTrajectory(t,y)
    if (length(t)<2) keyboard; end
    traj = PPTrajectory(spline(t,y));
    % todo: check for discrete outputs, and handle them by making a zoh
    % for discrete and cubic for continuous, then returning a
    % MixedTrajectory object.  Or just manually set the pp values for
    % higher orders to zero for the dt variables.
  end

if (nargout>0)
  if (getNumOutputs(obj)<1) error('this dynamical system doesn''t have any outputs'); end
  
  if (isDT(obj))
    error('dt outputs not handled yet (but it should be trivial');
  else
    % note: could make this more exact.  see comments in bug# 623.
    pstruct.SaveFormat = 'StructureWithTime';
    pstruct.SaveTime = 'on';
    pstruct.TimeSaveName = 'tout';
    pstruct.SaveState = 'off';
    pstruct.SaveOutput = 'on';
    pstruct.OutputSaveName = 'yout';
    pstruct.Refine = '3';
    
    simout = sim(mdl,pstruct);
    
    t = simout.get('tout');
    y = simout.get('yout').signals.values';
    
    % find any zero-crossing events (they won't have the extra refine steps)
    zcs = find(diff(t)<1e-10);
    if (length(zcs)>0) % then we have a hybrid trajectory
      zcs = [0;zcs;length(t)];
      pptraj = {};
      for i=1:(length(zcs)-1)
        inds = (zcs(i)+1):zcs(i+1);
        if (length(inds)<2) % successive zero crossings.
          % commands like this are useful for debugging here:
          %  plot(y(2,:),y(3,:),'.-',y(2,zcs(2:end)),y(3,zcs(2:end)),'r*')
          %  keyboard;
          if (i>0) warning('successive zero crossings'); end
          continue;
        end
        pptraj = {pptraj{:},makeSubTrajectory(t(inds),y(:,inds))};
      end
      if (length(pptraj)>1)
        traj = HybridTrajectory(pptraj);
      else
        traj = pptraj{1};
      end
    else
      traj = makeSubTrajectory(t,y);
    end
  end
else
  sim(mdl,pstruct);
end

end
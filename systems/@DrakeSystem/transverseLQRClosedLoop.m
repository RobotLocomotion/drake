function [controller,sys,xtraj,utraj,Vtraj,Vf]=transverseLQRClosedLoop(obj,xtraj,utraj,Q,R,Vf,transSurf)

[controller,Vtraj] = transverseLQR(obj,xtraj,utraj,Q,R,Vf,transSurf);

sys = feedback(obj,controller);

N = obj.getNumStates();
m = controller.getNumContStates();
if (controller.getNumDiscStates()~=0), error('not implemented yet'); end

if (m>0)
  xtraj = MixedTrajectory({xtraj, PPTrajectory(foh(xtraj.tspan,xtraj.tspan))},{1:N,N+(1:m)}); % add controller state
end
utraj = FunctionHandleTrajectory(@(t)zeros(0),[0 0],xtraj.tspan);  % utraj is now empty

Pi = getPi(transSurf,xtraj.tspan(end));
Vf = Pi*Vf*Pi';

% Note: unlike tvlqrClosedLoop, Vtraj does not have to change due to the 
% extra state variable from control (tau), since it is tacked on the end.
% Same for Vf.


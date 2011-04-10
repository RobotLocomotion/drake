classdef LQRTree < HybridRobotLibSystem

  properties
    ticontroller
    Vf
    tvcontrollers={};  
    Vtv={};
    p_x
  end
  
  methods
    function obj=LQRTree(tilqr,V)
      typecheck(tilqr,'LTIControl');
      typecheck(V,'msspoly');
      
      obj = setModeOutputFlag(obj,false);
      
      obj.ticontroller = tilqr;
      obj.Vf = V;
      obj.p_x=decomp(V);
      
      nU = length(tilqr.u0);
      obj = addMode(obj,ConstantControl(zeros(nU,1)));
      
      obj = addMode(obj,tilqr);
      lqr_in = inline('(x-obj.x0)''*obj.S*(x-obj.x0) - obj.rho','obj','t','junk','x');

      % switch from zero control to any controller
      obj = addTransition(obj,1,@inAnyFunnelGuard,@transitionIntoAnyFunnel,true,true);
      
      % switch from ti controller to any controller
      obj = addTransition(obj,2,@(obj,~,~,x)1-double(subs(obj.Vf,obj.p_x,x-obj.ticontroller.x0)),@transitionIntoAnyFunnel,true,true);
    end
    
    function [obj,ind]=addTrajectory(obj,tvlqr,Vtv,parentID,parentT)
      typecheck(tvlqr,'LTVControl');
      typecheck(Vtv,'PolynomialTrajectory');
      
      N = length(obj.tvcontrollers);
      if (nargin<4) parentID=2; end
      typecheck(parentID,'double');
      if (parentID<2 || parentID>(N+2)) error('invalid parent ID'); end
      
      % todo: check here that trajectories for the controller and lyapunov
      % function match?
      
      obj.tvcontrollers = {obj.tvcontrollers{:},tvlqr};  % note: i'm storing everything twice here
      obj.Vtv = {obj.Vtv{:},Vtv};
      
      [obj,mode_num] = addMode(obj,tvlqr);
      
      % fell out of this funnel
      obj = addTransition(obj,mode_num,@(obj,t,t0,x)1-Vtv.polyeval(t-t0,x-tvlqr.x0.eval(t-t0)),@transitionIntoAnyFunnel,true,false);
      
      function [xn,to_mode,status]=toParent(~,~,~,~,~)
        to_mode=parentID;
        status=0;
        if (parentID==2) % ti
          xn=[];
        elseif (parentID>2) % tv
          xn=parentT;
        end
      end
      
      % from this controller to the parent
      obj = addTransition(obj,mode_num,@(obj,t,t0,x) Vtv.tspan(end)-t+t0,@toParent,false,false);
    end
  
    function phi=inAnyFunnelGuard(obj,t,~,x)  % note: this is expensive for a guard
      phi = checkFunnels(obj,x)-1;
    end
    
    function [t0,to_mode_num,status]=transitionIntoAnyFunnel(obj,m,t,~,x)
      status=0;
      [Vmin,to_mode_num,tmin]=checkFunnels(obj,x);
      if (Vmin>1) % then I'm not in any funnel
        t0=[];
        to_mode_num=1;  % put me in the zero controller
      elseif to_mode_num==2
        t0=[];  % don't need (can't have) the starting time for the ti controller
      else % landing in a tv funnel
        t0=t-tmin;
      end
    end
    
    function [Vmin,mode,tmin] = checkFunnels(obj,x)
      % find lowest V for this x, by searching over ti and tv funnels.
      
      % check ti funnel first
      Vmin=doubleSafe(subs(obj.Vf,obj.p_x,x-obj.ticontroller.x0));
      mode=2;
      tmin=0;
      
      % now check the tv funnels (note: could make this faster by precomputing)
      for i=1:length(obj.tvcontrollers)
        ts = obj.Vtv{i}.getBreaks();
        for j=1:length(ts)
          V(j)=obj.Vtv{i}.polyeval(ts(j),x-obj.tvcontrollers{i}.x0.eval(ts(j)));
        end
        [Vm,ind] = min(V);
        if (Vm<Vmin)
          Vmin=Vm;
          mode=i+2;
          tmin=ts(ind);
        end
      end
      % todo: perform a line search here to improve tmin
    end
    
  end % end methods

end


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end

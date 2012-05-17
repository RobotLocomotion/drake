function testFeedback

load robotlib_config;
oldpath=addpath([conf.root,'/examples/Pendulum']);

try 
  p = PendulumPlant();
  c = PendulumLQR(p);
  
  sys1=feedback(p,c);
  typecheck(sys1,'RobotLibSystem');
  
  p2 = SimulinkModel(p.getModel());
  p2 = setInputFrame(p2,p.getInputFrame());
  p2 = setStateFrame(p2,p.getStateFrame());
  p2 = setOutputFrame(p2,p.getOutputFrame());
  sys2=feedback(p2,c);
  typecheck(sys2,'SimulinkModel');
  
  for i=[];%1:100
    t=rand; x=randn(2,1); u=randn;
    if any(abs(dynamics(sys1,t,x,u) - dynamics(sys2,t,x,u))>1e-10)
      i
      x
      u
      xdot_robotlibsys=dynamics(sys1,t,x,u)
      xdot_simulinkmodel=dynamics(sys2,t,x,u)
      error('dynamics should match');
    end
    
    if (i<20)  % these are slower
      y1=simulate(sys1,[0 1],x);
      y2=simulate(sys2,[0 1],x);
      if (any(abs(y1.eval(1)-y2.eval(1))>1e-10))
        error('simulations should match');
      end
    end
  end

  % polynomial feedback 
  p = setInputLimits(p,-inf,inf);  % todo: another case where I get rid of this
  p = taylorApprox(p,0,[pi;0],0,3);
  sys1=feedback(p,c);
  typecheck(sys1,'PolynomialSystem');
  
  p2 = SimulinkModel(p.getModel());
  p2 = setInputFrame(p2,p.getInputFrame());
  p2 = setStateFrame(p2,p.getStateFrame());
  p2 = setOutputFrame(p2,p.getOutputFrame());
  sys2=feedback(p2,c);
  typecheck(sys2,'SimulinkModel');
  
  for i=1:100
    t=rand; x=randn(2,1); u=randn;
    if any(abs(dynamics(sys1,t,x,u) - dynamics(sys2,t,x,u))>1e-10)
      i
      x
      u
      xdot_polysys=dynamics(sys1,t,x,u)
      xdot_simulinkmodel=dynamics(sys2,t,x,u)
      error('dynamics should match');
    end
    
    if (i<20)  % these are slower
      y1=simulate(sys1,[0 1],x);
      y2=simulate(sys2,[0 1],x);
      if (any(abs(y1.eval(1)-y2.eval(1))>1e-10))
        error('simulations should match');
      end
    end
  end
  
catch ex
  path(oldpath);
  rethrow(ex);
end

path(oldpath);

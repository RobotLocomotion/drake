function trigPolyTest

checkDependency('spotless');
q=msspoly('q',2);
s=msspoly('s',2);
c=msspoly('c',2);

x=TrigPoly(q,s,c);
t=msspoly('t',1);

shouldFail('sin(x(1)^2)');
shouldFail('sin(t)');
shouldFail('sin(x(1)+t)');

shouldBeEqual(sin(x(1)+x(2)),s(1)*c(2)+s(2)*c(1));
shouldBeEqual(sin(x(1)+pi),-s(1))
shouldBeEqual(sin(x(1)),s(1))
shouldBeEqual(cos(x(2)),c(2))
shouldBeEqual(sin([x(2);4]),[s(2);sin(4)])
shouldBeEqual(q(1)-sin(x(1)),q(1)-s(1)) % shouldn't evaluate as valid, but should parse
shouldBeEqual(sin(x(1))-x(1),s(1)-q(1)) % shouldn't evaluate as valid, but should parse
shouldBeEqual(sin(x).^2+cos(x).^2,s.^2+c.^2)
shouldBeEqual(diff(sin(x),x),diag(cos(x)))
shouldBeEqual(diff(cos(x),x),diag(-sin(x)))
shouldBeEqual(diff(sin(x).^2+cos(x).^2,x),msspoly(zeros(2)))

root = getDrakePath();

try
  oldpath = addpath([root,'/examples/Pendulum']);

  p = PendulumPlant();
  tp=extractTrigPolySystem(p,struct('replace_output_w_new_state',true));
  checkDynamics(p,tp);

  path(oldpath);
  oldpath = addpath([root,'/examples/Acrobot']);

  p=AcrobotPlant();
  tp=extractTrigPolySystem(p,struct('replace_output_w_new_state',true));
  checkDynamics(p,tp);

  path(oldpath);
  oldpath = addpath([root,'/examples']);

  p=VanDerPol();
  pp=extractTrigPolySystem(p);
  [prhs,plhs]=getPolyDynamics(p);
  [pprhs,pplhs]=getPolyDynamics(pp);
  shouldBeEqual(prhs,pprhs);
  shouldBeEqual(plhs,pplhs);

%  p=SineSys();
%  pp=p.stereographicProjection()

catch ex
  path(oldpath);
  rethrow(ex);
end

path(oldpath)

end

function shouldFail(str)

try
  eval(str)
catch ex
  return
end

str
error('should have failed on that test');

end

function shouldBeEqual(p1,p2)
  if isempty(p1)&&isempty(p1)
    % that's ok, too
    return;
  end

  if isempty(p1) || isempty(p2) || any(~isequal(clean(p1,1e-12),clean(p2,1e-12)))
    p1
    p2
    error('p1 ~= p2.  something is wrong here');
  end
end


function checkDynamics(p,tp)
  for i=1:20
    t=rand;
    x=Point(p.getStateFrame,randn(p.getNumStates,1));
    u=randn;
    xdotp = p.dynamics(t,double(x),u);
    xtp=double(x.inFrame(tp.getStateFrame));
    xdottp = tp.dynamics(t,xtp,u);
    tf = findTransform(tp.getStateFrame,p.getStateFrame);
    [~,dtf] = geval(@output,tf,t,[],xtp);
    xdottpinp = dtf(:,2:end)*xdottp;
    valuecheck(xdotp,xdottpinp);
  end
end

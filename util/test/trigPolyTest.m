function trigPolyTest

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

load drake_config;
oldpath = addpath([conf.root,'/examples/Pendulum']);

try 
  p = PendulumPlant();
  p.makeTrigPolySystem(struct('replace_output_w_new_state',true));
  % todo:  check that it's correct...? this just checks that it runs

  path(oldpath);
  oldpath = addpath([conf.root,'/examples/Acrobot']);

  p=AcrobotPlant();
  p.makeTrigPolySystem(struct('replace_output_w_new_state',true));
  % todo:  check that it's correct...? this just checks that it runs
  
  path(oldpath);
  oldpath = addpath([conf.root,'/examples']);
  
  p=VanDerPol();
  pp=p.makeTrigPolySystem();
  if (~isequal(pp.p_dynamics,p.p_dynamics)) 
    error('should return the original system if called on a polynomial system'); 
  end
  
  p=sinsys();
  pp=p.stereographicProjection()
  
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
  if any(~isequal(clean(p1,1e-12),clean(p2,1e-12)))
    p1
    p2
    error('p1 ~= p2.  something is wrong here');
  end
end

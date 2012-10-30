function coordinateTest(is,js)

if (nargin<1) is=1:10; end
if (nargin<2) js=1:2; end

for i=is
for j=js
  
if ismember(i,[1:3,7:8])
  options.view = 'right';
elseif ismember(i,[4:6,9:10])
  options.view = 'front';
else
  options.view = 'top';
end
m = PlanarRigidBodyModel(['p',num2str(i),'.urdf'],options);
if even(j)
  m.gravity=[-9.81;0];
end
r = PlanarRigidBodyManipulator(m);
xtraj = r.simulate([0 5],randn(2,1));

if (nargin>0)
  v = r.constructVisualizer();
  v.xlim = .7*[-1 1];
  v.ylim = .7*[-1 1];
  v.playback(xtraj);
end

xf = eval(xtraj,5);

m.doKinematics(xf(1));
[~,c] = m.body(2).getInertial();

comf = m.body(2).forwardKin(c);

fprintf(1,'i:%d,j:%d,c: %s, a: %s, v: %s, g: %s, s: %d, cf: %s.\n',i,j,mat2str(c),mat2str(m.body(2).joint_axis), options.view, mat2str(m.gravity), m.body(2).jsign, mat2str(comf,1));
if (comf'*m.gravity<.4)
  fprintf(1,'grav check: ');
  cprintf('red','fail\n');
%  error('com should be drawn towards gravity!');
else
  fprintf(1,'grav check: pass\n');
end

m.doKinematics(pi/4);
postheta = m.body(2).forwardKin([1;0]);
if valuecheck(m.body(2).joint_axis,[0;1;0])
  pthetaOK=postheta(2)<0;
elseif valuecheck(m.body(2).joint_axis,[0;-1;0])
  pthetaOK=postheta(2)>0;
elseif valuecheck(m.body(2).joint_axis,[1;0;0])
  pthetaOK=postheta(2)>0;
elseif valuecheck(m.body(2).joint_axis,[-1;0;0])
  pthetaOK=postheta(2)<0;
else
  error('axis not handled');
end

if pthetaOK
  fprintf(1,'pos theta check: pass\n');
else
  fprintf(1,'pos theta check: ');
  cprintf('red','fail\n');
end

fprintf(1,'\n');
end

end
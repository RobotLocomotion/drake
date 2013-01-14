function model = PlanarNLink(N)
% Constructs a planar rigid-body model with N links.  The first link
% is unactuated; the remaining are all torque-actuated. 
%
% @param N the number of links.  N must be >=1
% @retval sys a PlanarRigidBodyManipulator with N links.  
%

% NOTEST

if (N<1) error('N must be >=1'); end

model = PlanarRigidBodyManipulator();
model.name = ['Planar',num2str(N),'Link'];

% add the base
body=PlanarRigidBody();
body.linkname='base';
body.Ttree=[-1,0,0; 0,1,0; 0,0,1];
model.body=body;

% first link
model = addLink(model,1,1,.05);
for i=2:N
  model = addLink(model,1.2,1.2,.05);
end

model = compile(model);

end


function model=addLink(model,mass,len,radius)

  body=PlanarRigidBody();
  ind=length(model.body)+1;

  % link properties
  body.linkname=['link',num2str(ind-1)];
  setInertial(body,mass,[0;-len/2],mass*len^2/12);  % solid rod w/ uniform mass
  body.geometry{1}.x = radius*[-1 1 1 -1 -1];
  body.geometry{1}.y = len*[0 0 -1 -1 0];
  h=figure(1035); set(h,'Visible','off');
  co = get(gca,'ColorOrder');
  close(h);
  body.geometry{1}.c = co(mod(ind-2,size(co,1))+1,:);
  model.body = [model.body, body];
    
  % joint properties
  if (ind>2)
    parentlen=-min(model.body(ind-1).geometry{1}.y);
  else
    parentlen = 0;
  end
  model = addJoint(model,['joint',num2str(ind-1)],'revolute',model.body(ind-1),body,[0;0;-parentlen],zeros(3,1),model.view_axis,.1);
  
  if (ind>2)  % leave the first joint as passive
    actuator = RigidBodyActuator();
    actuator.name = ['joint',num2str(ind),'_torque'];
    actuator.joint=body;
    model.actuator = [model.actuator; actuator];
  end
end

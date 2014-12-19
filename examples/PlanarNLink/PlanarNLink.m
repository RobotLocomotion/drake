classdef PlanarNLink < PlanarRigidBodyManipulator
  
  methods
    function model = PlanarNLink(N) 
      % Constructs a planar rigid-body model with N links.  The first link
      % is unactuated; the remaining are all torque-actuated.
      %
      % @param N the number of links.  N must be >=1
      % @retval sys a PlanarRigidBodyManipulator with N links.
      %

      if (N<1) error('N must be >=1'); end

      model = model@PlanarRigidBodyManipulator();
      model.name{1} = ['Planar',num2str(N),'Link'];

      % add the base
      body=RigidBody();
      body.linkname='base';
      body.robotnum=1;
      model.body=body;
      
      % first link
      model = addPlanarLink(model,1,1,.05);
      for i=2:N
        model = addPlanarLink(model,1.2,1.2,.05);
      end
      
      model = compile(model);
    end
    
  end
  
  methods (Access=private)
    function model=addPlanarLink(model,mass,len,radius)
      
      body=RigidBody();
      ind=length(model.body)+1;
      
      % link properties
      body.linkname=['link',num2str(ind-1)];
      body.robotnum=1;
      body = setInertial(body,mass,[0;0;-len/2],diag([1;mass*len^2/12;1]));  % solid rod w/ uniform mass
      body.visual_geometry{1} = RigidBodyCylinder(radius,len);
      body.visual_geometry{1}.T = [eye(3),[0 0 -len/2]';zeros(1,3),1];
      h=figure(1035); set(h,'Visible','off');
      co = get(gca,'ColorOrder');
      close(h);
      body.visual_geometry{1}.c = co(mod(ind-2,size(co,1))+1,:);
      model.body = [model.body, body];
      
      % joint properties
      if (ind>2)
        parentlen=model.body(ind-1).visual_geometry{1}.len;
      else
        parentlen = 0;
      end
      limits.joint_limit_min = -Inf;
      limits.joint_limit_max = Inf;
      limits.effort_min = -10;
      limits.effort_max = 10;
      limits.velocity_limit = Inf;
      model = addJoint(model,['joint',num2str(ind-1)],'revolute',ind-1,ind,[0;0;-parentlen],zeros(3,1),model.view_axis,.1,[],[],[],limits);
      
      if (ind>2)  % leave the first joint as passive
        actuator = RigidBodyActuator();
        actuator.name = ['joint',num2str(ind),'_torque'];
        actuator.joint=ind;
        model.actuator = [model.actuator; actuator];
      end
    end
  end
end



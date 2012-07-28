classdef PlanarRigidBodyManipulator < Manipulator
  % This class wraps the planar pieces of the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties (SetAccess=private,GetAccess=private)  
    model;     % RigidBodyModel object
  end
  
  methods
    function obj = PlanarRigidBodyManipulator(model)
      obj = obj@Manipulator(0,0);

      if (nargin<1)
        urdf_filename=uigetfile('*.urdf');
      elseif ischar(model)
        obj.model = RigidBodyModel.parseURDF(model);
      elseif isa(model,'RigidBodyModel')
        obj.model = model;
      else
        error('model must be a RigidBodyModel or the name of a urdf file'); 
      end

      obj = obj.setNumInputs(size(obj.model.B,2));
      obj = obj.setNumDOF(obj.model.featherstone.NB);
      obj = obj.setNumOutputs(2*obj.model.featherstone.NB);
      
      if (length(obj.model.loop)>0 || size([obj.model.body.ground_contact],2)>0)
        error('haven''t reimplemented position and velocity constraints yet'); 
      end
%      obj = obj.setNumPositionConstraints(2*length(obj.model.loop)+size([obj.model.body.ground_contact],2));
%      obj = obj.setNumVelocityConstraints(0);%size([obj.model.body.ground_contact],2));
    end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      m = obj.model.featherstone;
      [H,C] = HandCp(m,q,qd,{},obj.model.gravity);
      C=C+m.damping'.*qd;
      B = obj.model.B;
    end

    function phi = positionConstraints(obj,q)
      error('not implemented yet');  % need to pull from posadev
      phi=[];

      % handle kinematic loops
      % note: each loop adds two constraints 
      for i=1:length(obj.model.loop)
        % for each loop, add the constraints on T1(q) and T2(q), // todo: finish this
        % where
        % T1 is the transformation from the least common ancestor to the
        % constraint in link1 coordinates
        % T2 is the transformation from the least common ancester to the
        % constraint in link2 coordinates
        loop=obj.model.loop(i);
        
        T1 = loop.T1;
        b=loop.body1;
        while (b~=loop.least_common_ancestor)
          TJ = Tjcalcp(b.jcode,q(b.dofnum));
          T1 = b.Ttree*TJ*T1;
          b = b.parent;
        end
        T2 = loop.T2;
        b=loop.body2;
        while (b~=loop.least_common_ancestor)
          TJ = Tjcalcp(b.jcode,q(b.dofnum));
          T2 = b.Ttree*TJ*T2;
          b = b.parent;
        end
        
        if (loop.jcode==1)  % pin joint adds constraint that the transformations must match in position at the origin
          phi = [phi; [1,0,0; 0,1,0]*(T1*[0;0;1] - T2*[0;0;1])];
        else
          error('not implemented yet');
        end
      end

      % handle ground contacts (note: treating all of these as active constraints; should only get here for the active ones)
      gc_body_inds = find(~cellfun(@isempty,{obj.model.body.ground_contact}));
      if (~isempty(gc_body_inds)) obj.model = doKinematics(obj.model,q,0*q); end  % note: this is potentially inefficient when automatically computing jacobians
      for i=gc_body_inds;
        % adding constraint that z position of ground contact = groundProfile for all contact points
        body = obj.model.body(i);
        pts = body.T*[body.ground_contact; ones(1,size(body.ground_contact,2))]; %[0;0;1];
        phi = [phi; pts(2,:)-obj.groundProfile(pts(1,:))];
      end

    end

    function psi = velocityConstraints(obj,q,qd)
      error('not implemented yet');  % need to pull from posadev

      psi=[];
      
      % implement ground contact xdot = 0 constraints
      gc_body_inds = find(~cellfun(@isempty,{obj.model.body.ground_contact}));
      if (isempty(gc_body_inds))
        return;
      end
      
      obj.model = doKinematics(obj.model,q,qd);  % very inefficient!
      for i=gc_body_inds;
        % adding constraint that z position of ground contact = groundProfile for all contact points
        body = obj.model.body(i);
        abspts = body.T(2:3,2:3)*body.ground_contact;
        vpts = body.v(2:3) + body.v(1)*[-abspts(2);abspts(1)];
        psi = [psi; vpts(2,:)];
      end
    end
    
    function z=groundProfile(obj,x)
      z=0*x;
    end
    
    function v=constructVisualizer(obj)
      v = PlanarRigidBodyVisualizer(obj.model);
    end
  end
  
end


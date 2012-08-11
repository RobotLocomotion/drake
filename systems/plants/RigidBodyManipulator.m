classdef RigidBodyManipulator < Manipulator
  % This class wraps the planar pieces of the spatial vector library (v1) 
  % provided by Roy Featherstone on his website: 
  %   http://users.cecs.anu.edu.au/~roy/spatial/documentation.html
    
  properties (SetAccess=private,GetAccess=private)  
    model;     % RigidBodyModel object
  end
  
  methods
    function obj = RigidBodyManipulator(model)
      obj = obj@Manipulator(0,0);

      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        obj.model = RigidBodyModel(fullfile(pathname,filename));
      elseif ischar(model)
        obj.model = RigidBodyModel(model);
      elseif isa(model,'RigidBodyModel')
        obj.model = model;
      else
        error('model must be a RigidBodyModel or the name of a urdf file'); 
      end

      obj = obj.setNumInputs(size(obj.model.B,2));
      obj = obj.setNumDOF(obj.model.featherstone.NB);
      obj = obj.setNumOutputs(2*obj.model.featherstone.NB);
      
      if getNumInputs(obj)>0
        inputframe = CoordinateFrame([obj.model.name,'Input'],getNumInputs(obj));
        inputframe = setCoordinateNames(inputframe,{obj.model.actuator.name}');
        obj = setInputFrame(obj,inputframe);
      end

      stateframe = CoordinateFrame([obj.model.name,'State'],2*obj.model.featherstone.NB,'x');
      joints={obj.model.body(~cellfun(@isempty,{obj.model.body.parent})).jointname}';
      stateframe = setCoordinateNames(stateframe,vertcat(joints,cellfun(@(a) [a,'dot'],joints,'UniformOutput',false)));
      obj = setStateFrame(obj,stateframe);
      obj = setOutputFrame(obj,stateframe);  % output = state
      
      if (length(obj.model.loop)>0 || size([obj.model.body.ground_contact],2)>0)
        error('haven''t reimplemented position and velocity constraints yet'); 
      end
%      obj = obj.setNumPositionConstraints(2*length(obj.model.loop)+size([obj.model.body.ground_contact],2));
%      obj = obj.setNumVelocityConstraints(0);%size([obj.model.body.ground_contact],2));
    end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      m = obj.model.featherstone;
      [H,C] = HandC(m,q,qd,{},obj.model.gravity);
      C=C+m.damping'.*qd;
      B = obj.model.B;
    end

    function v=constructVisualizer(obj)
      v = RigidBodyWRLVisualizer(obj.getStateFrame,obj.model);
    end
  end
  
end


classdef HybridPlanarRigidBodyManipulator < HybridDrakeSystem
    
  methods
    function obj=HybridPlanarRigidBodyManipulator(model)
      % construct the (single) mode of this system:
      S=warning('off','Drake:PlanarRigidBodyManipulatorWContact:ShouldBeAHybridMode');
      mode = PlanarRigidBodyManipulatorWContact(model);
      warning(S);
      
      obj = obj@HybridDrakeSystem(mode.getNumInputs(), mode.getNumOutputs());
      obj = setInputFrame(obj,getInputFrame(mode));
      obj = setOutputFrame(obj,getOutputFrame(mode));
      
      obj = addMode(obj,mode);
      
      % todo: add self-transitions
    end
    
    function v = constructVisualizer(obj)
      v = obj.modes{1}.constructVisualizer;
    end
  end
  
  
end
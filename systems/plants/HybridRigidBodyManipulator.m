classdef ContactHybridRigidBodyManipulator < HybridDrakeSystem
  methods
    function obj = ContactHybridRigidBodyManipulator(p)
      obj = obj@HybridDrakeSystem(p.getNumInputs,p.getNumStates);
      obj = setInputFrame(obj,p.getInputFrame);
      obj = setOutputFrame(obj,p.getOutputFrame);
      
      nC = p.getNum
    end
  end
end
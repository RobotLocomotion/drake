classdef TimeSteppingRigidBodySensorWithState < TimeSteppingRigidBodySensor
  % Sensors that have internal dynamics we want to model, or sensors that
  % need to be prevented from being direct feedthrough.
  
  methods (Abstract=true)
    fr = constructStateFrame(obj,tsmanip);
    x0 = getInitialState(obj,tsmanip);
    [xdn,df] = update(obj,tsmanip,t,x,u);
  end
end
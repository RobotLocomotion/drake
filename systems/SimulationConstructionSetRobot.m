classdef SimulationConstructionSetRobot < RobotLibSystem
% Implements the DynamicalSystem interface for a SimulationConstructionSet robot.
% SimulationConstructionSet is developed by IHMC.

  % constructor
  methods
    function obj = SimulationConstructionSetRobot(robotobj)
      % Construct the system
      %
      % @param robotobj a com.yobotics.simulationconstructionset.Robot java
      % object
      %
      % Note: In order to create the robot object, you'll need to make sure 
      % that you have all of the right jar files in your path.
      % Example:
      %
      % eclipse_root = '/Users/russt/Documents/workspace/';
      % javaaddpath([eclipse_root,'ThirdParty/ThirdPartyJars/Java3d/j3dcore.jar']);
      % javaaddpath([eclipse_root,'ThirdParty/ThirdPartyJars/Java3d/j3dutils.jar']);
      % javaaddpath([eclipse_root,'ThirdParty/ThirdPartyJars/Java3d/vecmath.jar']);
      % javaaddpath([eclipse_root,'SimulationConstructionSet/classes']);
      % javaaddpath([eclipse_root,'IHMCUtilities/classes']);
      % javaaddpath([eclipse_root,'Plotting/classes']);
      % javaaddpath([eclipse_root,'DoublePendulum/classes']);

      % todo: compute correct number of states and inputs here
      obj=obj@RobotLibSystem(0,0,0,0,0,1);
      
      if (~isa(robotobj,'com.yobotics.simulationconstructionset.Robot'))
        error('Must pass in a simulation construction set robot java object');
      end
      obj.robotobj=robotobj;
      
      obj = setNumContStates(obj,obj.robotobj.getStateVectorLength());
      obj = setNumOutputs(obj,obj.getNumStates());
      obj = setNumInputs(obj,obj.robotobj.getInputVectorLength());
    end
  end
  
  % default methods - these should be implemented or overwritten
  % 
  methods
    function x0 = getInitialState(obj)
      x0 = zeros(obj.getNumStates(),1); 
    end
    
    function [xcdot,df] = dynamics(obj,t,x,u)
      if (nargout<2)
        xcdot = obj.robotobj.matlabDynamics(x,u);
      else
        nX = obj.getNumStates();
        nU = obj.getNumInputs();
        m = obj.robotobj.matlabDynamicsAndGradients(x,u);
        xcdot = m(1:nX);
        df = [zeros(nX,1), reshape(m(nX+(1:nX*nX)),nX,nX), reshape(m(nX+nX*nX+(1:nX*nU)),nX,nU)];
      end
    end
    
    function xdn = update(obj,t,x,u)
      error('discrete states not supported yet.  shouldn''t get here.');
    end
    
    function [y,dy] = output(obj,t,x,u)
      y = x;
      if (nargout>0)
        nY = obj.getNumOutputs();
        dy = [zeros(nY,1),eye(nY),zeros(nY,obj.getNumInputs())];
      end
    end
  end

  properties (SetAccess=private, GetAccess=public)
    robotobj;   % a instance of the robot java object
  end
  
end

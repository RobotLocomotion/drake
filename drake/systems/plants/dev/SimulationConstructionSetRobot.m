classdef SimulationConstructionSetRobot < DrakeSystem
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
      obj=obj@DrakeSystem(0,0,0,0,0,1);
      
      checkDependency('simulationconstructionset');
      
      typecheck(robotobj,'com.yobotics.simulationconstructionset.Robot');
      obj.robotobj=robotobj;
      
      obj = setNumContStates(obj,obj.robotobj.getStateVectorLength());
      obj.numGCstates = obj.robotobj.getDiscStateVectorLength();
      obj = setNumOutputs(obj,obj.robotobj.getContStateVectorLength());
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
        xcdot = [obj.robotobj.matlabDynamics(t,x,u);zeros(obj.numGCstates,1)];
      else
        nX = obj.getNumStates();
        nXc = nX - obj.numGCstates;
        nU = obj.getNumInputs();
        m = obj.robotobj.matlabDynamicsAndGradients(t,x,u);
        xcdot = [m(1:nXc);zeros(obj.numGCstates,1)];
        df = [[zeros(nXc,1), reshape(m(nXc+(1:nXc*nX)),nXc,nX), reshape(m(nXc+nXc*nX+(1:nXc*nU)),nXc,nU)];zeros(obj.numGCstates,1+nX+nU)];
      end
    end
    
    function xdn = update(obj,t,x,u)
      error('discrete states not supported yet.  shouldn''t get here.');
    end
    
    function [y,dy] = output(obj,t,x,u)
      nY = obj.getNumOutputs();
      y = x(1:nY);
      if (nargout>0)
        dy = [zeros(nY,1),eye(nY),zeros(nY,obj.numGCstates+obj.getNumInputs())];
      end
    end
  end

  properties (SetAccess=private, GetAccess=public)
    robotobj;   % a instance of the robot java object
    numGCstates;
  end
  
end

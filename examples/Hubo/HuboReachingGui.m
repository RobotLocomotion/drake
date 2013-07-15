classdef HuboReachingGui < PlanarRigidBodyVisualizer
% This is the planar Hubo visualizer with two outputs which represent the
% last point on the screen that was clicked (in world coordinates).  It
% should be run as a DynamicalSytem (e.g. connected with a hubo state
% input and the output should go to a controller which drives the hand
% position).  
  
  methods
    function obj = HuboReachingGui(r)
      if nargin>0
        if isa(r,'TimeSteppingRigidBodyManipulator')
          r = r.manip;
        end
        typecheck(r,'PlanarRigidBodyModel');
      else
        options.floating = true;
        options.view = 'right';
        m = PlanarRigidBodyModel('Hubo_description/urdf/jaemiHubo.urdf',options);
        r = TimeSteppingRigidBodyManipulator(m,.0005);
      end
      obj = obj@PlanarRigidBodyVisualizer(r.getStateFrame(),m,options);
      obj.display_dt=0.05;
      
      obj = setNumDiscStates(obj,2);
      obj = setStateFrame(obj,CoordinateFrame('LastMouseClick',2,'m',{'x','z'}));
      
      obj = setNumOutputs(obj,2);
      obj = setOutputFrame(obj,CoordinateFrame('DesiredRightHandPosition',2,'h',{'x','z'}));
    end
    
    function pt0 = getInitialState(obj)
      obj.draw(0,double(Point(obj.getInputFrame)));  % open up the figure
      pt = get(get(32,'CurrentAxes'),'CurrentPoint');
      pt0 = [pt(1,1);pt(1,2)];
    end
    
    function pt0 = update(obj,t,pt0,x)
      % intentionally do nothing.  pt0 is a constant state.
    end
    
    function pt = output(obj,t,pt0,x)
      output@PlanarRigidBodyVisualizer(obj,t,[],x);  % draws (and calls kinematics)
      
      pt = get(get(32,'CurrentAxes'),'CurrentPoint');
      pt = [pt(1,1);pt(1,2)];
      if all(pt==pt0)  % then I haven't actually received a click yet
        % instead, return the current position of the hand
        b=findLink(obj.model,'RPalm');
        pt = forwardKin(b,[0;0]);
      end
    end
  end
end
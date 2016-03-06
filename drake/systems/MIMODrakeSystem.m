classdef MIMODrakeSystem < DrakeSystem
  % For DrakeSystems with MultiCoordinateFrame inputs and/or outputs, this
  % method simply provides a helper routine which splits up the input into
  % pieces and reassembles the output.
  
  methods
    function obj = MIMODrakeSystem(num_xc,num_xd,inputFrame,outputFrame,direct_feedthrough_flag,time_invariant_flag)
      typecheck(inputFrame,'CoordinateFrame');
      sizecheck(inputFrame,1);
      typecheck(outputFrame,'CoordinateFrame');
      sizecheck(outputFrame,1);
      
      obj = obj@DrakeSystem(num_xc,num_xd,inputFrame.dim,outputFrame.dim,direct_feedthrough_flag,time_invariant_flag);
      obj = setInputFrame(obj,inputFrame);
      obj = setOutputFrame(obj,outputFrame);
      
      % cache these for efficiency:
      obj.b_input_is_multiframe = isa(inputFrame,'MultiCoordinateFrame');
      obj.b_output_is_multiframe = isa(outputFrame,'MultiCoordinateFrame');
      if obj.b_output_is_multiframe
        obj.mimo_y = cell(1,length(outputFrame.frame));
      end
    end
    
    function xcdot = mimoDynamics(obj,t,x,varargin)
      error('Drake:MIMODrakeSystem:AbstractMethod','MIMO systems with continuous states must overload this method');
    end
    
    function xdn = mimoUpdate(obj,t,x,varargin)
      error('Drake:MIMODrakeSystem:AbstractMethod','MIMO systems with discrete states must implement Update (ie overload update function)');
    end
    
    function varargout = mimoOutput(obj,t,x,varargin)
      error('Drake:MIMODrakeSystem:AbstractMethod','MIMO systems with outputs must overload this method.');
    end
    
    function zcs = mimoZeroCrossings(obj,t,x,varargin)
      error('Drake:MIMODrakeSystem:AbstractMethod','MIMO systems with zero crossings must implement the zeroCrossings method'); 
    end
  end
  
  methods (Sealed=true) % don't allow people to overload these... they should overload the mimo versions (above) instead.

    % todo: pass gradients through, etc.
    function xcdot = dynamics(obj,t,x,u)
      if obj.b_input_is_multiframe
        mimo_u = splitCoordinates(obj.getInputFrame(),u);
        xcdot = mimoDynamics(obj,t,x,mimo_u{:});
      else
        xcdot = mimoDynamics(obj,t,x,u);
      end
    end
    
    function xdn = update(obj,t,x,u)
      if obj.b_input_is_multiframe
        mimo_u = splitCoordinates(getInputFrame(obj),u);
        xdn = mimoUpdate(obj,t,x,mimo_u{:});
      else
        xdn = mimoUpdate(obj,t,x,u);
      end
    end
    
    function y = output(obj,t,x,u)
      if obj.direct_feedthrough_flag
        if obj.b_input_is_multiframe
          mimo_u = splitCoordinates(getInputFrame(obj),u);
        else
          mimo_u = {u};
        end
      else
        mimo_u={};
      end
      
      if obj.b_output_is_multiframe
        [obj.mimo_y{:}] = mimoOutput(obj,t,x,mimo_u{:});
        y = mergeCoordinates(getOutputFrame(obj),obj.mimo_y);
      else
        y = mimoOutput(obj,t,x,mimo_u{:});
      end      
    end
    
    function zcs = zeroCrossings(obj,t,x,u)
      if pbj.b_input_is_multiframe
        mimo_u = splitCoordinates(obj.getInputFrame(),u);
        zcs = mimoZeroCrossings(obj,t,x,mimo_u{:});
      else
        zcs = mimoZeroCrossings(obj,t,x,u);
      end      
    end
  end
  
  properties
    b_input_is_multiframe    % cache these checks so I don't need to do e.g. isa(getInputFrame(obj),'MultiCoordinateFrame') in the main simulation loop
    b_output_is_multiframe
    mimo_y
  end
  
end
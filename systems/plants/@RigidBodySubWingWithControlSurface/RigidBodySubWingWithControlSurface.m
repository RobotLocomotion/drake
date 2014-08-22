classdef RigidBodySubWingWithControlSurface < RigidBodySubWing
  % Implements functionality similar to RigidBodySubWing but with a
  % control surface attached to the wing.
      
  properties
    fCl;
    fCd;
    fCm;
    chord;
    span;
    stall_angle;
  end
  
  methods
    
    function obj = RigidBodySubWingWithControlSurface(frame_id, profile, chord, span, stall_angle, velocity, control_surface)
      % Constructor taking similar arguments to RigidBodySubWing except
      % with the addition of a ControlSurface
      %
      % @param control_surface a ControlSurface attached to this wing.
      
      typecheck(frame_id,'numeric');
      obj.kinframe = frame_id;
      linux = isunix();
      obj.area = chord*span;
      obj.chord = chord;
      obj.span = span;
      obj.stall_angle = stall_angle;
      
      % compute the coefficients for the flat plate
      
      [obj.fCl, obj.fCd, obj.fCm] = obj.flatplate();
      
      
      
    end
    
    
    function [fCl, fCd, fCm] = flatplate(obj)
      disp('Using a flat plate airfoil with control surfaces.')
        laminarpts = 30;
        stallAngle = obj.stall_angle;
        
        angles = [-180:2:-(stallAngle+.0001) -stallAngle:2*stallAngle/laminarpts:(stallAngle-.0001) stallAngle:2:180];
        %CMangles is used to make the Moment coefficient zero when the wing
        %is not stalled
        CMangles = [-180:2:-(stallAngle+.0001) zeros(1,laminarpts) stallAngle:2:180];
        obj.fCm = foh(angles, -(CMangles./90)*obj.rho*obj.area*obj.chord/4);
        obj.fCl = spline(angles, .5*(2*sind(angles).*cosd(angles))*obj.rho*obj.area);
        obj.fCd = spline(angles, .5*(2*sind(angles).^2)*obj.rho*obj.area);
      
        % TODO
    end
    
    
  end
  
  
end

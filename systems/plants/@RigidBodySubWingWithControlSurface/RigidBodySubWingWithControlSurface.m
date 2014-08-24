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
      
    end
    
    function [ fCl, fCd, fCm ] = flatplate(obj, aoa, control_surface_angle_rad)
      % Computes coefficients for a flat plate given angle of attack and
      % the control surface offset in radians.
      %
      % @param aoa angle of attack of the wing (can be an array)
      % @param control_surface_angle_rad angle of the control surface in
      %   radians.  0 is no deflection, positive deflection is upwards (if
      %   it was an elevator, it would be pitch up on the plane)
      %   can be an array.
      %
      % 
      % Lift force = \f$ \frac{1}{2} \rho v^2 \left( C_l(aoa) S + C_l(aoa+u) S_2 \right) \f$
      % <pre>
      %   rho: air pressure
      %   S: wing area
      %   S2: control surface area
      %   aoa: angle of attack
      %   u: amount of deflection in radians from the control input
      %   v: airspeed
      % </pre>
      %
      % See pages 34-35 of Cory10a.
      %
      % So here, we return \f$ \frac{1}{2}*\rho* (C_l(aoa) * S + C_l(aoa+u) * S_2) \f$
      % because then you can multiply by just \f$ v^2 \f$ to compute force.
      %   
      % @retval fCl instantaneous life force divided by \f$ v^2 \f$
      % @retval Cd instantaneous coefficient of drag divided by \f$ v^2 \f$
      % @retval Cm instantaneous moment coefficient
      
      Cl = 2 * sin(aoa) * cos(aoa);
      Cl_control_surface = 2 * sin(aoa + control_surface_angle_rad) * cos(aoa + control_surface_angle_rad);
      
      Cd = 2 * (sin(aoa)) ^ 2;
      Cd_control_surface = 2 * (sin(aoa + control_surface_angle_rad)) ^ 2;
      
      control_surface_area = obj.control_surface.span * obj.control_surface.chord;
      
      fCl = 0.5 * obj.rho * ( Cl * obj.area + Cl_control_surface * control_surface_area);
      
      fCd = 0.5 * obj.rho * ( Cd * obj.area + Cd_control_surface * control_surface_area);
      
      fCm
      
      
      
      
    end
    
    
  end
  
  
end

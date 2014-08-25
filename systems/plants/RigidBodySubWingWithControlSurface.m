classdef RigidBodySubWingWithControlSurface < RigidBodyForceElement
  % Implements functionality similar to RigidBodySubWing but with a
  % control surface attached to the wing.
      
  properties
    kinframe;  % index to RigidBodyFrame
    fCl  % PPTrajectories (splines) representing the *dimensional* coefficients
    fCd  % with fCl = 1/2 rho*S*Cl, etc. (S=area)
    fCm
    dfCl
    dfCd
    dfCm
    area
    %Air density for 20 degC dry air, at sea level
    rho = 1.204;
    span;
    stall_angle;
    chord;
    control_surface % the control surface attached to this wing
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
      obj.control_surface = control_surface;
      
      % compute the coefficients for the flat plate
      
      [obj.fCl, obj.fCd, obj.fCm] = obj.flatplate_old();
      
      laminarpts = 30;
      aoa_range = [-180:2:-(obj.stall_angle+.0001) -obj.stall_angle:2*obj.stall_angle/laminarpts:(obj.stall_angle-.0001) obj.stall_angle:2:180];
      aoa_range = deg2rad(aoa_range);
      
      
      
      [fCl2, fCd2, fCm2] = obj.flatplate(aoa_range, 0);
      
      keyboard
      
    end
    
    
    function [fCl, fCd, fCm] = flatplate_old(obj)
      disp('Using a flat plate airfoil with control surfaces.')
        laminarpts = 30;
        stallAngle = obj.stall_angle;
        
        angles = [-180:2:-(stallAngle+.0001) -stallAngle:2*stallAngle/laminarpts:(stallAngle-.0001) stallAngle:2:180];
        %CMangles is used to make the Moment coefficient zero when the wing
        %is not stalled
        CMangles = [-180:2:-(stallAngle+.0001) zeros(1,laminarpts) stallAngle:2:180];
        fCm = foh(angles, -(CMangles./90)*obj.rho*obj.area*obj.chord/4);
        fCl = spline(angles, .5*(2*sind(angles).*cosd(angles))*obj.rho*obj.area);
        %fCd = spline(angles, .5*(2*sind(angles).^2)*obj.rho*obj.area);
        fCd = (.5*(2*sind(angles).^2)*obj.rho*obj.area);
      
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
      %
      % Drag force = \f$ \frac{1}{2} \rho v^2 \left( C_d(aoa) S + C_d(aoa+u) S_2 \right) \f$
      %
      % Moment torque comes just from the lift on the control surface since
      % the center of lift is at the center of a flat plate.
      %
      % Moment torque = \f$ v^2 \rho r \sin(aoa + u) \cos(aoa + u) S_2 \f$
      %
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
      % @retval Cm instantaneous moment coefficient divided by \f$ v^2 \f$
      
      Cl = 2 .* sin(aoa) .* cos(aoa);
      Cl_control_surface = 2 .* sin(aoa + control_surface_angle_rad) .* cos(aoa + control_surface_angle_rad);
      
      Cd = 2 .* (sin(aoa)) .^ 2;
      Cd_control_surface = 2 .* (sin(aoa + control_surface_angle_rad)) .^ 2;
      
      control_surface_area = obj.control_surface.span .* obj.control_surface.chord;
      
      fCl = 0.5 .* obj.rho .* ( Cl .* obj.area + Cl_control_surface .* control_surface_area);
      
      fCd = 0.5 .* obj.rho .* ( Cd .* obj.area + Cd_control_surface .* control_surface_area);
      
      % distance from the center of the wing to the center of the control
      % surface
      r = obj.chord / 2 + obj.control_surface.chord / 2;
      
      fCm = obj.rho .* r .* sin(aoa + control_surface_angle_rad) .* cos(aoa + control_surface_angle_rad) .* control_surface_area;
      
      
      
      
    end
    
    
  end
  
  
end

classdef RigidBodySubWingWithControlSurface < RigidBodySubWing
  % Implements functionality similar to RigidBodySubWing but with a
  % control surface attached to the wing.
      
  properties
    span;
    stall_angle;
    chord;
    control_surface % the control surface attached to this wing
    fCl_control_surface;
    fCd_control_surface;
    fCm_control_surface; 
  end
  
  methods
    
    function obj = RigidBodySubWingWithControlSurface(frame_id, profile, chord, span, stall_angle, velocity, control_surface)
      % Constructor taking similar arguments to RigidBodySubWing except
      % with the addition of a ControlSurface
      %
      % @param control_surface a ControlSurface attached to this wing.
      
      obj = obj@RigidBodySubWing(frame_id, profile, chord, span, stall_angle, velocity);

      obj.chord = chord;
      obj.span = span;
      obj.stall_angle = stall_angle;
      obj.control_surface = control_surface;
      obj.direct_feedthrough_flag = true;
      
      % compute the coefficients for the flat plate
      
      %[obj.fCl, obj.fCd, obj.fCm] = obj.flatplate_old();
      
      
      [obj.fCl_control_surface, obj.fCd_control_surface, obj.fCm_control_surface ] = obj.flateplateControlSurfaceInterp();
      
      
      
      
    end
    
    function [force, B_force, dforce, dB_force] = computeSpatialForce(obj,manip,q,qd)
      % Computes the forces from the wing including the control surface.
      % Returns the force from the wing along with the B matrix which the
      % matrix for a linearized input for the control surface.
      %
      % @param manip
      % @param q
      % @param qd
      %
      %
      % @retval force force from the wing that is independant of the
      %   control surface
      %
      % @retval B_force B matrix containing the linearized component of the
      %   force from the input (from the control surface's deflection)
      
      
      % first, call the parent class's  computeSpatialForce to get the
      % u-invariant parts
      
      [force, dforce] = computeSpatialForce@RigidBodySubWing(manip, q, qd);
      
      % now compute B and dB
      
      % get the coefficients for this point
      
      [wingvel_world, dwingvel_worlddq, dwingvel_worlddqd] = obj.computeWingVelocity(manip, q, qd);
      
      airspeed = norm(wingvel_world);
      
      aoa = -(180/pi)*atan2(wingvel_rel(3),wingvel_rel(1));
      
      Cl = obj.fCl_control_surface(aoa, control_surface_rad);
      
      
      
    
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
        fCd = spline(angles, .5*(2*sind(angles).^2)*obj.rho*obj.area);
        
      
    end
    
    function [ fCl_interp, fCd_interp, fCm_interp ] = flateplateControlSurfaceInterp(obj)
      % Builds a smooth interpolation of values for lift, drag, and moment
      % coefficients for a flatplate control surface
      %
      %
      % See flateplateControlSurface for more information on the
      % computation.
      %
      % @retval fCl_interp smooth interpolated surface for lift force
      %   divided by \f$ v^2 \f$
      % @retval fCd_interp smooth interpolated surface for drag force
      %   divided by \f$ v^2 \f$
      % @retval fCm_interp smooth interpolated surface for moment force
      %   divided by \f$ v^2 \f$
      
      
      % build the ranges for interpolation
      laminarpts = 30;
      aoa_range = [-180:2:-(obj.stall_angle+.0001) -obj.stall_angle:2*obj.stall_angle/laminarpts:(obj.stall_angle-.0001) obj.stall_angle:2:180];
      aoa_range = deg2rad(aoa_range);
      
      control_surface_increment = 0.01; % in radians
      
      control_surface_range = obj.control_surface.min_deflection : control_surface_increment : obj.control_surface.max_deflection;
      
      [ fCl, fCd, fCm, aoa_mat, control_surface_mat] = obj.flatplateControlSurface(aoa_range, control_surface_range);
      
      
      fCl_interp = scatteredInterpolant(aoa_mat(:), control_surface_mat(:), fCl(:));
      fCd_interp = scatteredInterpolant(aoa_mat(:), control_surface_mat(:), fCd(:));
      fCm_interp = scatteredInterpolant(aoa_mat(:), control_surface_mat(:), fCm(:));

    end
    
    function [ fCl, fCd, fCm, aoa_mat, control_surface_mat ] = flatplateControlSurface(obj, aoa, control_surface_angle_rad)
      % Computes coefficients for a flat plate control surface given angle of attack and
      % the control surface offset in radians.
      %
      % @param aoa angle of attack of the wing (can be an array)
      % @param control_surface_angle_rad angle of the control surface in
      %   radians.  0 is no deflection, positive deflection is upwards (if
      %   it was an elevator, it would be pitch up on the plane)
      %   can be an array.
      %
      % 
      % Lift force from control surface = \f$ \frac{1}{2} \rho v^2 C_l(aoa+u) S_2 \f$
      %
      % Drag force = \f$ \frac{1}{2} \rho v^2 C_d(aoa+u) S_2 \f$
      %
      % Moment torque comes just from the lift on the control surface since
      % drag is in the plane and will not produce a torque
      %
      % Moment torque = \f$ v^2 \rho r \sin(aoa + u) \cos(aoa + u) S_2 \f$
      %
      % <pre>
      %   rho: air pressure
      %   S2: control surface area
      %   aoa: angle of attack
      %   u: amount of deflection in radians from the control input
      %   v: airspeed
      % </pre>
      %
      % See pages 34-35 of Cory10a.
      %
      % So here, we return \f$ \frac{1}{2} \rho C_l(aoa+u) S_2 \f$
      % because then you can multiply by just \f$ v^2 \f$ to compute force.
      %   
      % @retval fCl instantaneous life force from the control surface divided by \f$ v^2 \f$
      % @retval Cd instantaneous drag force from the control surface divided by \f$ v^2 \f$
      % @retval Cm instantaneous moment coefficient from the control surface divided by \f$ v^2 \f$
      % @retval aoa_mat matrix of angle of attack values used 
      % @retval control_surface_mat matrix of control surfaces used
      
      % repmat so we evaluate at every aoa and control surface angle
      
      aoa_mat = repmat(aoa, length(control_surface_angle_rad), 1);
      control_surface_mat = repmat(control_surface_angle_rad', 1, length(aoa));
      
      Cl_control_surface = 2 .* sin(aoa_mat + control_surface_mat) .* cos(aoa_mat + control_surface_mat);
      
      Cd_control_surface = 2 .* (sin(aoa_mat + control_surface_mat)) .^ 2;
      
      control_surface_area = obj.control_surface.span .* obj.control_surface.chord;
      
      fCl = 0.5 .* obj.rho .* Cl_control_surface .* control_surface_area;
      
      fCd = 0.5 .* obj.rho .* Cd_control_surface .* control_surface_area;
      
      % distance from the center of the wing to the center of the control
      % surface
      r = obj.chord / 2 + obj.control_surface.chord / 2;
      
      fCm = obj.rho .* r .* sin(aoa_mat + control_surface_mat) .* cos(aoa_mat + control_surface_mat) .* control_surface_area;
      
      
    end
    
    
    
  end
  
  
end

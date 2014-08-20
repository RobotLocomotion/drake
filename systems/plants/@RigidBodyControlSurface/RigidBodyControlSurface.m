classdef RigidBodyControlSurface < RigidBodyWing
  % Implements massless, position controlled control surfaces.  This is an
  % approximation of small aircraft control surfaces that assumes
  % instantaneous response from the servo motor and control surface.
  %
  % It is useful because it can significantly reduce the number of states
  % in a model by eliminating the states from aileron, elevator, or rudder
  % position and velocity.
  
  properties

  end

  methods
    function obj = RigidBodyControlSurface(frame_id, profile, chord, span, stallAngle, velocity)
      %calls AVL and XFOIL over different angles of attack at the
      %given velocity, generates first order polynomials of the CL,
      %CD, and pitch moments of the wing.  The axes for use in DRAKE are:
      %   X = forward, in the usual direction of travel
      %   Y = out th
      
      
      
    end
    
    
  end
  
  
end
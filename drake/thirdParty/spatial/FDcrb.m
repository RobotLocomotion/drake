function  qdd = FDcrb( model, q, qd, tau, f_ext, grav_accn )

% FDcrb  Forward Dynamics via Composite-Rigid-Body Algorithm
% FDcrb(model,q,qd,tau,f_ext,grav_accn) calculates the forward dynamics of
% a kinematic tree via the composite-rigid-body algorithm.  q, qd and tau
% are vectors of joint position, velocity and force variables; and the
% return value is a vector of joint acceleration variables.  f_ext is a
% cell array specifying external forces acting on the bodies.  If f_ext ==
% {} then there are no external forces; otherwise, f_ext{i} is a spatial
% force vector giving the force acting on body i, expressed in body i
% coordinates.  Empty cells in f_ext are interpreted as zero forces.
% grav_accn is a 3D vector expressing the linear acceleration due to
% gravity.  The arguments f_ext and grav_accn are optional, and default to
% the values {} and [0,0,-9.81], respectively, if omitted.

if nargin == 4
  [H,C] = HandC( model, q, qd );
elseif nargin == 5
  [H,C] = HandC( model, q, qd, f_ext );
else
  [H,C] = HandC( model, q, qd, f_ext, grav_accn );
end

qdd = H \ (tau - C);

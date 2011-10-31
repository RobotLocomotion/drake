function  qdd = FDcrbp( model, q, qd, tau, f_ext, grav_accn )

% FDcrbp  Forward Dynamics (planar) via Composite-Rigid-Body Algorithm.
% FDcrbp(model,q,qd,tau,f_ext,grav_accn) calculates the forward dynamics of
% a kinematic tree via the composite-rigid-body algorithm, evaluated using
% planar vectors.  q, qd and tau are vectors of joint position, velocity
% and force variables; and the return value is a vector of joint
% acceleration variables.  f_ext is a cell array specifying external forces
% acting on the bodies.  If f_ext == {} then there are no external forces;
% otherwise, f_ext{i} is a planar force vector giving the force acting on
% body i, expressed in body i coordinates.  Empty cells in f_ext are
% interpreted as zero forces.  grav_accn is a 2D vector expressing the
% linear acceleration due to gravity in the x-y plane.  The arguments f_ext
% and grav_accn are optional, and default to zero (i.e., {} and [0 0],
% respectively) if omitted.

if nargin == 4
  [H,C] = HandCp( model, q, qd );
elseif nargin == 5
  [H,C] = HandCp( model, q, qd, f_ext );
else
  [H,C] = HandCp( model, q, qd, f_ext, grav_accn );
end

qdd = H \ (tau - C);

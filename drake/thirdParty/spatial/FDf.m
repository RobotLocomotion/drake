function  [afb,qdd] = FDf( model, Xfb, vfb, q, qd, tau, f_ext, grav_accn )

% FDf  Floating-Base Forward Dynamics via Articulated-Body Algorithm
% [afb,qdd]=FDf(model,Xfb,vfb,q,qd,tau,f_ext,grav_accn) calculates the
% forward dynamics of a floating-base kinematic tree, such as that created
% by the function floatbase (i.e., body 6 in the system model is the
% floating base), via the articulated-body algorithm.  Xfb is the
% coordinate transform from fixed to floating base coordinates; vfb is the
% spatial velocity of the floating base, expressed in fixed-base
% coordinates; and q, qd and tau contain the position, velocity and force
% variables for the real joints in the system (i.e., joints 7 onwards in
% the system model).  The return values are the spatial acceleration of the
% floating base, expressed in fixed-base coordinates, and the joint
% acceleration vector for the real joints.  f_ext is a cell array
% specifying external forces acting on the bodies.  If f_ext == {} then
% there are no external forces; otherwise, f_ext{i} is a spatial force
% vector giving the force acting on body i, expressed in body i
% coordinates.  Thus, f_ext{6} is the force acting on the floating base,
% and f_ext{1} to f_ext{5} are ignored.  Empty cells in f_ext are
% interpreted as zero forces.  grav_accn is a 3D vector expressing the
% linear acceleration due to gravity.  The arguments f_ext and grav_accn
% are optional, and default to the values {} and [0,0,-9.81], respectively,
% if omitted.  (Caution: vfb is expressed in fixed-base coordinates, but
% f_ext{6} is expressed in floating-base coordinates.)

if nargin < 8
  a_grav = [0;0;0;0;0;-9.81];
else
  a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

external_force = ( nargin > 6 && length(f_ext) > 0 );

vfb = Xfb * vfb;

NBR = model.NB - 6;			% NB & parent array for Rest of model
parentR = model.parent(7:model.NB) - 6;

for i = 1:NBR
  [ XJ, S{i} ] = jcalc( model.pitch(i+6), q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i+6};
  if parentR(i) == 0
    v{i} = Xup{i}*vfb + vJ;
  else
    v{i} = Xup{i}*v{parentR(i)} + vJ;
  end
  c{i} = crm(v{i}) * vJ;
  IA{i} = model.I{i+6};
  pA{i} = crf(v{i}) * IA{i} * v{i};
  if external_force && length(f_ext{i+6}) > 0
    pA{i} = pA{i} - f_ext{i+6};
  end
end

IAfb = model.I{6};
pAfb = crf(vfb) * IAfb * vfb;
if external_force && length(f_ext{6}) > 0
  pAfb = pAfb - f_ext{6};
end

for i = NBR:-1:1
  U{i} = IA{i} * S{i};
  d{i} = S{i}' * U{i};
  u{i} = tau(i) - S{i}'*pA{i};
  Ia = IA{i} - U{i}/d{i}*U{i}';
  pa = pA{i} + Ia*c{i} + U{i} * u{i}/d{i};
  if parentR(i) == 0
    IAfb = IAfb + Xup{i}' * Ia * Xup{i};
    pAfb = pAfb + Xup{i}' * pa;
  else
    IA{parentR(i)} = IA{parentR(i)} + Xup{i}' * Ia * Xup{i};
    pA{parentR(i)} = pA{parentR(i)} + Xup{i}' * pa;
  end
end

afb = - IAfb \ pAfb;			% floating base accn without gravity

for i = 1:NBR
  if parentR(i) == 0
    a{i} = Xup{i} * afb + c{i};
  else
    a{i} = Xup{i} * a{parentR(i)} + c{i};
  end
  qdd(i,1) = (u{i} - U{i}'*a{i})/d{i};
  a{i} = a{i} + S{i}*qdd(i);
end

afb = Xfb \ afb + a_grav;		% true flt base accn in ref coords

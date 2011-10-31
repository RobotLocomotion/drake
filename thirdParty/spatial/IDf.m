function  [afb,tau] = IDf( model, Xfb, vfb, q, qd, qdd, f_ext, grav_accn )

% IDf  Floating-Base Inverse Dynamics
% [afb,tau]=IDf(model,Xfb,vfb,q,qd,qdd,f_ext,grav_accn) calculates the
% inverse dynamics of a floating-base kinematic tree, such as that created
% by the function floatbase (i.e., body 6 in the system model is the
% floating base).  Xfb is the coordinate transform from fixed to floating
% base coordinates; vfb is the spatial velocity of the floating base,
% expressed in fixed-base coordinates; and q, qd and qdd contain the
% position, velocity and acceleration variables for the real joints in the
% system (i.e., joints 7 onwards in the system model).  The return values
% are the spatial acceleration of the floating base, expressed in
% fixed-base coordinates, and the joint force vector for the real joints.
% f_ext is a cell array specifying external forces acting on the bodies.
% If f_ext == {} then there are no external forces; otherwise, f_ext{i} is
% a spatial force vector giving the force acting on body i, expressed in
% body i coordinates.  Thus, f_ext{6} is the force acting on the floating
% base, and f_ext{1} to f_ext{5} are ignored.  Empty cells in f_ext are
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
afb = Xfb * -a_grav;			% fictitious base acc for calc pC
% alternative: afb = zeros(6,1);

NBR = model.NB - 6;			% NB & parent array for Rest of model
parentR = model.parent(7:model.NB) - 6;

for i = 1:NBR
  [ XJ, S{i} ] = jcalc( model.pitch(i+6), q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i+6};
  if parentR(i) == 0
    v{i} = Xup{i}*vfb + vJ;
    a{i} = Xup{i}*afb + S{i}*qdd(i) + crm(v{i})*vJ;
  else
    v{i} = Xup{i}*v{parentR(i)} + vJ;
    a{i} = Xup{i}*a{parentR(i)} + S{i}*qdd(i) + crm(v{i})*vJ;
  end
  IC{i} = model.I{i+6};
  pC{i} = IC{i}*a{i} + crf(v{i})*IC{i}*v{i};
  if external_force && length(f_ext{i+6}) > 0
    pC{i} = pC{i} - f_ext{i+6};
  end
end

ICfb = model.I{6};
pCfb = ICfb*afb + crf(vfb)*ICfb*vfb;
if external_force && length(f_ext{6}) > 0
  pCfb = pCfb - f_ext{6};
end

for i = NBR:-1:1
  if parentR(i) == 0
    ICfb = ICfb + Xup{i}'*IC{i}*Xup{i};
    pCfb = pCfb + Xup{i}'*pC{i};
  else
    IC{parentR(i)} = IC{parentR(i)} + Xup{i}'*IC{i}*Xup{i};
    pC{parentR(i)} = pC{parentR(i)} + Xup{i}'*pC{i};
  end
end

afb = - ICfb \ pCfb;			% true base acceleration

for i = 1:NBR
  if parentR(i) == 0
    a{i} = Xup{i}*afb;
  else
    a{i} = Xup{i}*a{parentR(i)};
  end
  tau(i,1) = S{i}'*(IC{i}*a{i} + pC{i});
end

afb = inv(Xfb) * afb;
% alternative: afb = inv(Xfb) * afb + a_grav;

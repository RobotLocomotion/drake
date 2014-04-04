function  [H,C] = HandC(manipulator, q, v, f_ext, grav_accn )

% HandC  Calculate coefficients of equation of motion.
% [H,C]=HandC(model,q,v,f_ext,grav_accn) calculates the coefficients of
% the joint-space equation of motion, tau=H(q)vd+C(d,v,f_ext), where q,
% v and vd are the joint position, velocity and acceleration vectors, H
% is the joint-space inertia matrix, C is the vector of gravity,
% external-force and velocity-product terms, and tau is the joint force
% vector.  Algorithm: recursive Newton-Euler for C, and
% Composite-Rigid-Body for H.  f_ext is a cell array specifying external
% forces acting on the bodies.  If f_ext == {} then there are no external
% forces; otherwise, f_ext{i} is a spatial force vector giving the force
% acting on body i, expressed in body i coordinates.  Empty cells in f_ext
% are interpreted as zero forces.  grav_accn is a 3D vector expressing the
% linear acceleration due to gravity.  The arguments f_ext and grav_accn
% are optional, and default to the values {} and [0,0,-9.81], respectively,
% if omitted.
%
% UPDATED by russt:  f_ext is an empty or (sparse) 6 x model.NB matrix

if nargin < 5
  a_grav = [0;0;0;0;0;-9.81];
else
  a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

external_force = ( nargin > 3 && length(f_ext) > 0 );

kinsol = doKinematics(manipulator, q, true, false, v);
inertias_world = computeInertiasInWorld(manipulator, kinsol);
H = computeMassMatrix(manipulator, kinsol, inertias_world);
C = computeBiasTerm(manipulator, kinsol, f_ext, a_grav);

end

function H = computeMassMatrix(manipulator, kinsol, inertias_world)
% world frame implementation
NB = length(manipulator.body);
nv = length(kinsol.v);
H = zeros(nv, nv) * kinsol.q(1); % minor adjustment to make TaylorVar work better.
composite_inertias = computeCompositeRigidBodyInertias(manipulator, inertias_world);

for i = 2 : NB
  Ic = composite_inertias{i};
  Si = kinsol.J{i};
  i_indices = manipulator.body(i).velocity_num;
  F = Ic * Si;
  H(i_indices, i_indices) = Si' * F;
  
  j = i;
  while j ~= 2
    j = manipulator.body(j).parent;
    body_j = manipulator.body(j);
    j_indices = body_j.velocity_num;
    Sj = kinsol.J{j};
    Hji = Sj' * F;
    H(j_indices, i_indices) = Hji;
    H(i_indices, j_indices) = Hji';
  end
end

% old body frame implementation
% % minor adjustment to make TaylorVar work better.
% H=zeros(NB)*q(1);
% 
% for i = 1:NB
%   n = i;  % note to twan: this was n = model.position_num(i);
%   fh = IC{i} * S{i};
%   H(n,n) = S{i}' * fh;
%   j = i;
%   while featherstone.parent(j) > 0
%     fh = Xup{j}' * fh;
%     j = featherstone.parent(j);
%     np = j; % note to twan: this was np = model.position_num(j);
%     H(n,np) = S{j}' * fh;
%     H(np,n) = H(n,np);
%   end
% end
end

function ret = computeInertiasInWorld(manipulator, kinsol)
NB = length(manipulator.body);
ret = cell(NB, 1);
ret{1} = zeros(6, 6);
for i = 2 : NB
  body = manipulator.body(i);
  bodyToWorld = kinsol.T{i};
  worldToBody = homogTransInv(bodyToWorld);
  AdWorldToBody = transformAdjoint(worldToBody);
  ret{i} = AdWorldToBody' * body.I * AdWorldToBody;
end
end

function ret = computeCompositeRigidBodyInertias(manipulator, inertias_world)
% computes composite rigid body inertias expressed in world frame

NB = length(inertias_world);
ret = inertias_world;
for i = NB : -1 : 2
  body = manipulator.body(i);
  ret{body.parent} = ret{body.parent} + ret{i};
end

end

function C = computeBiasTerm(manipulator, kinsol, external_forces, gravitational_accel)
nv = length(kinsol.v);
root_accel = -gravitational_accel; % as if we're standing in an elevator that's accelerating upwards
spatial_accels = computeSpatialAccelerations(manipulator, kinsol.T, kinsol.twists, kinsol.q, kinsol.v, zeros(nv, 1), root_accel);

C = 0;

% old body frame implementation
% for i = 1:NB
%   body = manipulator.body(i + 1);
%   [ XJ, S{i} ] = jcalc(body, q(body.position_num) );
%   vJ = S{i}*v(body.velocity_num);
%   Xup{i} = XJ * featherstone.Xtree{i};
%   if featherstone.parent(i) == 0
%     v{i} = vJ;
%     avp{i} = Xup{i} * -gravitational_accel;
%   else
%     v{i} = Xup{i}*v{featherstone.parent(i)} + vJ;
%     avp{i} = Xup{i}*avp{featherstone.parent(i)} + crm(v{i})*vJ;
%   end
%   fvp{i} = featherstone.I{i}*avp{i} + crf(v{i})*featherstone.I{i}*v{i};
%   if external_force
%     fvp{i} = fvp{i} - f_ext(:,i);
%   end
% end
% 
% IC = featherstone.I;				% composite inertia calculation
% 
% C = zeros(NB,1)*q(1);
% 
% for i = NB:-1:1
%   n = i;  % note to twan: this was n = model.position_num(i);
%   C(n,1) = S{i}' * fvp{i};
%   if featherstone.parent(i) ~= 0
%     fvp{featherstone.parent(i)} = fvp{featherstone.parent(i)} + Xup{i}'*fvp{i};
%     IC{featherstone.parent(i)} = IC{featherstone.parent(i)} + Xup{i}'*IC{i}*Xup{i};
%   end
% end

end
function testIndividualCentersOfPressure()
addpath('..');

options.floating = true;
options.use_mex = true;
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% testTangentialTorque(r);
testMex(r);

end

function testTangentialTorque(r)
load('../data/atlas_fp.mat');
nq = r.getNumPositions();
q = xstar(1:nq);

sides = {'l', 'r'};
active_supports = nan(1, length(sides));
for i = 1 : length(sides)
  active_supports(i) = r.findLinkInd([sides{i} '_foot']);
end

kinsol = r.doKinematics(q, false, false);

[~,B,~,~,normals] = contactConstraintsBV(r,kinsol);
nbeta = sum(cellfun('size', B, 2));
beta = rand(nbeta, 1);
cops = individualCentersOfPressure(r, kinsol, active_supports, normals, B, beta);

ncontact_points = size(B, 2);
ncontact_points_per_foot = ncontact_points / length(active_supports);
nd = nbeta / ncontact_points;
beta_cell = num2cell(reshape(beta, nd, []), 1);

for i = 1 : length(sides)
  indices = (i - 1) * ncontact_points_per_foot + (1 : ncontact_points_per_foot);
  B_for_foot = B(indices);
  normals_for_foot = normals(:, indices);
  normal = normals_for_foot(:, 1);
  beta_cell_for_foot = beta_cell(indices);
  contact_point_forces = cellfun(@mtimes, B_for_foot, beta_cell_for_foot, 'UniformOutput', false);
  contact_point_forces = [contact_point_forces{:}];
  foot_force = sum(contact_point_forces, 2);
  contact_positions = r.getBody(active_supports(i)).getTerrainContactPoints();
  foot_torque = sum(cross(contact_positions, contact_point_forces), 2);
  foot_tangential_torque = foot_torque - normal * dot(normal, foot_torque);
  foot_cop_body = r.bodyKin(kinsol, active_supports(i), cops(:, i));
  foot_torque_due_to_force_at_cop = cross(foot_cop_body, foot_force);  
  foot_tangential_torque_back = foot_torque_due_to_force_at_cop - normal * dot(normal, foot_torque_due_to_force_at_cop);
  valuecheck(foot_tangential_torque, foot_tangential_torque_back);
end
end

function testMex(r)
nq = r.getNumPositions();
q = randn(nq, 1);
q(3) = -10; % make sure we're completely under the ground so that both feet are in contact.

sides = {'l', 'r'};
active_supports = nan(1, length(sides));
for i = 1 : length(sides)
  active_supports(i) = r.findLinkInd([sides{i} '_foot']);
end

kinsol = r.doKinematics(q, false, false);

[~,B,~,~,normals] = contactConstraintsBV(r,kinsol);
nbeta = sum(cellfun('size', B, 2));
beta = rand(nbeta, 1);
cops = individualCentersOfPressure(r, kinsol, active_supports, normals, B, beta);


[~] = r.doKinematics(q, false, true);

ncontact_points = size(B, 2);
nd = nbeta / ncontact_points;
ncontact_points_per_foot = ncontact_points / length(active_supports);

supp.bodies = active_supports;
supp.contact_pts = {1:ncontact_points_per_foot, 1:ncontact_points_per_foot};
supp.contact_groups = {{'heel', 'toe'}, {'heel', 'toe'}};
supp.num_contact_pts = [ncontact_points_per_foot ncontact_points_per_foot];
supp.contact_surfaces = 0*active_supports;

B = [B{:}];

cops_mex = individualCentersOfPressuremex(r.getMexModelPtr, supp, normals, nd, B, beta);
disp(cops);
disp(cops_mex);
% valuecheck(cops, cops_mex);

end

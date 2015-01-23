function terrain_pts = sampleSwingTerrain(biped, step1, step2, contact_width, options)
% Sample the terrain between two poses of the robot's foot and compute a terrain profile.
% @param biped a Biped
% @param step1 a Footstep, the original footstep location
% @param step2 a Footstep, the final footstep location
% @param contact_width the effective width of the foot, from biped.contactVolume()
% @option nrho number of points to scan in the direction perpendicular to travel
% @option nlambda number of points to scan in the direction of travel
% @retval terrain_pts a 2xN matrix. The first row is distance along the line from step1 to step2;
%                     the second row is the max terrain height across the sampled width at that distance

if nargin < 5; options = struct(); end
if ~isfield(options, 'nrho'); options.nrho = 10; end

step_dist_xy = sqrt(sum((step2.pos(1:2) - step1.pos(1:2)).^2));
if ~isfield(options, 'nlambda'); options.nlambda = max([ceil(step_dist_xy / 0.02),3]); end

lambda_hat = (step2.pos(1:2) - step1.pos(1:2)) / step_dist_xy;

rho_hat = [0, -1; 1, 0] * lambda_hat;

terrain_pts = zeros(2, options.nlambda);
lambdas = linspace(0, step_dist_xy, options.nlambda);
rhos = linspace(-contact_width, contact_width, options.nrho);
[R, L] = meshgrid(rhos, lambdas);
xy = bsxfun(@plus, step1.pos(1:2), bsxfun(@times, reshape(R, 1, []), rho_hat) + bsxfun(@times, reshape(L, 1, []), lambda_hat));
z = biped.getTerrainHeight(xy);
z = reshape(z, size(R));
z = medfilt2(z, 'symmetric');
terrain_pts(2, :) = max(z, [], 2);
terrain_pts(1,:) = lambdas;

terrain_pts(2,1) = step1.pos(3);
terrain_pts(2,end) = step2.pos(3);

end

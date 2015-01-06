function terrain_pts = sampleSwingTerrain(biped, step1, step2, contact_width, options)

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

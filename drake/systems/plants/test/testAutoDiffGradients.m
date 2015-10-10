function testAutoDiffGradients()
r = createAtlas('rpy');
q = r.getRandomConfiguration();
options.grad_method = {'user', 'taylorvar'};
[~, ~] = geval(1, @(q) gevalFun(r, q), q, options);
end

function [H, dH] = gevalFun(r, q)
user_gradients = nargout > 1;
if user_gradients
  kinsol = r.doKinematics(q, [], struct('compute_gradients', true));
  [H, dH] = massMatrixmex(r.mex_model_ptr, kinsol.mex_ptr, 1);
else
  cache_ptr = createKinematicsCacheAutoDiffmex(r.mex_model_ptr, false);
  kinsol = r.doKinematics(q, TaylorVar.empty(0, 1), struct('kinematics_cache_ptr_to_use', cache_ptr));
  H = massMatrixmex(r.mex_model_ptr, kinsol.mex_ptr, 0);
end
end

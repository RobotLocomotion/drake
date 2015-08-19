function testGeometryGradientsComparison()

if exist('testGeometryGradientsmex','file')~=3
  error('Drake:MissingDependency', 'Cannot find testGeometryGradientsmex. It may not have been compiled due to a missing dependency.');
end

twist_size = 6;
nv = 8;
nq = 15;
X_cols = 5;

for i = 1 : 100
  T = [uniformlyRandomRotmat(), randn(3, 1); zeros(1, 3), 1];
  S = randn(twist_size, nv);
  qdot_to_v = randn(nv, nq);
  X = randn(twist_size, X_cols);
  dX = randn(numel(X), nq);
  x = randn(4, 1);
  
  [dT, dTInv, dAdT, dAdTinv_transpose, x_norm, dx_norm, ddx_norm] = testGeometryGradientsmex(T, S, qdot_to_v, X, dX, x);
  
  check_dT = dHomogTrans(T, S, qdot_to_v);
  check_dTInv = dHomogTransInv(T, dT);
  check_dAdT = dTransformSpatialMotion(T, X, dT, dX);
  check_dAdTinv_transpose = dTransformSpatialForce(T, X, dT, dX);
  [check_x_norm, check_dx_norm, check_ddx_norm] = normalizeVec(x);
  
  valuecheck(check_dT, dT);
  valuecheck(check_dTInv, dTInv);
  valuecheck(check_dAdT, dAdT);
  valuecheck(check_dAdTinv_transpose, dAdTinv_transpose);
  valuecheck(check_x_norm, x_norm);
  valuecheck(check_dx_norm, dx_norm);
  valuecheck(check_ddx_norm, reshape(ddx_norm, size(check_ddx_norm)));
end

end
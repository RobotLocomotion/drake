function testDTransformAdjointTranspose()
twist_size = 6;

for i = 1 : 100
  R = axis2rotmat(uniformlyRandomAxisAngle());
  p = randn(3, 1);
  T = [R, p; zeros(1, 3), 1];
  nq = 15;
  nv = 7;
  S = randn(twist_size, nv);
  qdotToV = randn(nv, nq);
  dT = dHomogTrans(T, S, qdotToV);
  X = randn(twist_size, 7);
  dX = randn(numel(X), nq);
  
  A = dTransformAdjointTranspose(T, X, dT, dX);
  AdT = transformAdjoint(T);
  dAdT = dTransformAdjoint(T, eye(twist_size), dT, zeros(twist_size^2, nq));
  B = matGradMultMat(AdT', X, transposeGrad(dAdT, size(AdT)), dX);
  
  valuecheck(A, B, 1e-12);
end
end
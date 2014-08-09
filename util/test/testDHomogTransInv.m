function testDHomogTransInv()
for i = 1 : 100
  R = axis2rotmat(uniformlyRandomAxisAngle());
  p = randn(3, 1);
  T = [R, p; zeros(1, 3), 1];
  nq = 15;
  nv = 7;
  S = randn(6, nv);
  qdotToV = randn(nv, nq);
  dT = dHomogTrans(T, S, qdotToV);
  dTInv = dHomogTransInv(T, dT);
  dTInv_check = invMatGrad(T, dT);
  valuecheck(dTInv_check, dTInv, 1e-10);
end
end
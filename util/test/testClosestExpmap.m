function testClosestExpmap()
% test the degenerate case when w2 is identity
theta1 = 2*pi*rand();
axis1 = randn(3,1);
axis1 = axis1/norm(axis1);
w1 = theta1*axis1;
w2 = zeros(3,1);
[w2_closest,dw2_closest_dw2] = closestExpmap(w1,w2);
if(theta1<pi)
  valuecheck(w2_closest,zeros(3,1));
else
  valuecheck(w2_closest,axis1*2*pi,1e-5);
end

% test non-degenerate case
for i = 1:100
  theta1 = 10*pi*rand();
  axis1 = randn(3,1);
  axis1 = axis1/norm(axis1);
  theta2 = 20*pi*rand();
  axis2 = randn(3,1);
  axis2 = axis2/norm(axis2);
  testClosestExpmap_userfun(theta1*axis1,theta2*axis2);
end

% test when w2 is very small, thus the gradient dw1_dw2 is big
axis1 = randn(3,1);
axis1 = axis1/norm(axis1);
theta1 = 4*pi + rand();
axis2 = randn(3,1);
axis2 = axis2/norm(axis2);
theta2 = 1e-3*rand(); 
testClosestExpmap_userfun(theta1*axis1,theta2*axis2);
end

function testClosestExpmap_userfun(w1,w2)
w1_theta = norm(w1);
w2_theta = norm(w2);
w1_axis = w1/w1_theta;
w2_axis = w2/w2_theta;
w2_closest_k = quadprog(4*pi^2,2*pi*w2_axis'*(w2-w1),[],[],[],[]);
w2_closest_k1 = floor(w2_closest_k);
w2_closest_k2 = ceil(w2_closest_k);
w2_closest1 = w2 + 2*pi*w2_closest_k1*w2_axis;
w2_closest2 = w2 + 2*pi*w2_closest_k2*w2_axis;
[w2_closest,dw2_closest_dw2] = closestExpmap(w1,w2);
w2_closest_mex = closestExpmapmex(w1,w2);
valuecheck(w2_closest_mex,w2_closest,1e-4);
[w2_closest_mex,dw2_closest_dw2_mex] = closestExpmapmex(w1,w2);
valuecheck(w2_closest_mex,w2_closest,1e-4);
valuecheck(dw2_closest_dw2_mex,dw2_closest_dw2,1e-4);
if(norm(w2_closest1-w1)<norm(w2_closest2-w1))
  valuecheck(w2_closest1,w2_closest,1e-3);
else
  valuecheck(w2_closest2,w2_closest,1e-3);
end
valuecheck(abs(expmap2quat(w2)'*expmap2quat(w2_closest)),1,1e-3);
[~,dw2_closest_dw2_numeric] = geval(@(w2) closestExpmap(w1,w2),w2,struct('grad_method','numerical','da',norm(w2)*1e-7));
tol = 1e-2;
if(~valuecheck(dw2_closest_dw2,dw2_closest_dw2_numeric,tol))
  gradient_err = dw2_closest_dw2 - dw2_closest_dw2_numeric;
  gradient_err(abs(gradient_err(:)) <= tol) = 0;
  gradient_err_normalize = gradient_err./dw2_closest_dw2;
  gradient_err_normalize(isnan(gradient_err_normalize(:))) = 0;
  if(any(abs(gradient_err_normalize(:)) > 5e-2))
    error('The analytical gradient of closestExpmap does not match the numeric gradient');
  end
end
end

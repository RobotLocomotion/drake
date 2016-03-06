function testCpp

tmp = addpathTemporary(fullfile(pwd,'..'));

r_matlab = AcrobotPlant;
r_cpp = AcrobotPlantCpp;

options.replace_output_w_new_state = true;
r_matlab_tp = extractTrigPolySystem(r_matlab,options);
r_cpp_tp = extractTrigPolySystem(r_cpp,options);
[am,bm] = getPolyDynamics(r_matlab_tp);
[ac,bc] = getPolyDynamics(r_cpp_tp);
assert(isequal(am,ac));
assert(isequal(bm,bc));

balanceLQR(r_cpp);

geval_options.grad_method = 'taylorvar';
for i=1:20
  t = randn();
  x = randn(4,1);
  u = randn();

  [H_matlab,C_matlab,B_matlab] = manipulatorDynamics(r_matlab,x(1:2),x(3:4));
  [H_cpp,C_cpp,B_cpp] = manipulatorDynamics(r_cpp,x(1:2),x(3:4));
  
  [~, ~, ~, dH_matlab, dC_matlab, dB_matlab] = geval(3, @(x) manipulatorDynamics(r_matlab, x(1:2), x(3:4)), x, geval_options);
  [~, ~, ~, dH_cpp, dC_cpp, dB_cpp] = geval(3, @(x) manipulatorDynamics(r_cpp, x(1:2), x(3:4)), x, geval_options);
  
  valuecheck(H_cpp,H_matlab);
  valuecheck(C_cpp,C_matlab);
  valuecheck(B_cpp,B_matlab);
  
  valuecheck(dH_cpp,dH_matlab);
  valuecheck(dC_cpp,dC_matlab);
  valuecheck(dB_cpp,dB_matlab);
  
  valuecheck(dynamics(r_matlab,t,x,u),dynamics(r_cpp,t,x,u));
end


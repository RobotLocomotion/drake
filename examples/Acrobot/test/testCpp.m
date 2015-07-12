function testCpp

tmp = addpathTemporary(fullfile(pwd,'..'));

r_matlab = AcrobotPlant;
r_cpp = AcrobotPlantCpp;

options.replace_output_w_new_state = true;
r_matlab_tp = extractTrigPolySystem(r_matlab,options);

% fails until TrigPoly variables are supported
%r_cpp_tp = extractTrigPolySystem(r_cpp,options);

% currently fails because taylorvars get passed into the C++ from the geval 
% in Manipulator/linearize. 
% balanceLQR(r_cpp);  

for i=1:20
  t = randn();
  x = randn(4,1);
  u = randn();

  [H_matlab,C_matlab,B_matlab] = manipulatorDynamics(r_matlab,x(1:2),x(3:4));
  [H_cpp,C_cpp,B_cpp] = manipulatorDynamics(r_cpp,x(1:2),x(3:4));
  
  valuecheck(H_cpp,H_matlab);
  valuecheck(C_cpp,C_matlab);
  valuecheck(B_cpp,B_matlab);
  
  valuecheck(dynamics(r_matlab,t,x,u),dynamics(r_cpp,t,x,u));
end


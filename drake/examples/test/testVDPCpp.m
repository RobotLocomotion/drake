function testVDPCpp

tmp = addpathTemporary(fullfile(pwd,'..'));

r_matlab = VanDerPol;
r_cpp = VanDerPolCpp;

xdot_matlab=getPolyDynamics(r_matlab);
xdot_cpp=getPolyDynamics(r_cpp);
assert(isequal(xdot_matlab,xdot_cpp))

for i=1:20
  t = randn();
  x = randn(2,1);
  u = [];

  valuecheck(dynamics(r_cpp,t,x,u),dynamics(r_matlab,t,x,u));
end


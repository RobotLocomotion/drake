function  [Xj,S] = jcalcp( code, q, jsign )

% jcalcp  Calculate joint transform and motion subspace (planar vectors).
% [XJ,S]=jcalcp(code,q) calculates the joint transform and motion subspace
% matrices for revolute (code==1), x-axis prismatic (code==2) and y-axis
% prismatic (code==3) joints.

if (nargin<3) jsign = 1; end
q = jsign*q;

if code == 1				% revolute joint
  Xj = Xpln( q, [0 0] );
  S = [1;0;0];
elseif code == 2			% x-axis prismatic joint
  Xj = Xpln( 0, [q 0] );
  S = [0;1;0];
elseif code == 3			% y-axis prismatic joint
  Xj = Xpln( 0, [0 q] );
  S = [0;0;1];
else
  error( 'unrecognised joint code' );
end

S = jsign*S;
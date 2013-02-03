function  dXj = djcalcp( code, q, jsign )

% jcalcp  Calculate derivative of joint transform
% [dXj]=djcalcp(code,q) calculates the joint transform derivative
% matrix for revolute (code==1), x-axis prismatic (code==2) and y-axis
% prismatic (code==3) joints.

if (nargin<3) jsign = 1; end
q = jsign*q;

if code == 1				% revolute joint
  dXj = dXpln( q, [0 0] ,1);
elseif code == 2			% x-axis prismatic joint
  dXj = dXpln( 0, [q 0] ,2);
elseif code == 3			% y-axis prismatic joint
  dXj = dXpln( 0, [0 q] ,3);
else
  error( 'unrecognised joint code' );
end

dXj = dXj*jsign;
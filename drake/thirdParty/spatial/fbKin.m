function  [o1, o2, o3] = fbKin( i1, i2, i3 )

% fbKin  Forward and Inverse Kinematics of Floating Base
% [X,v,a]=fbKin(q,qd,qdd) calculates the forward kinematics, and
% [q,qd,qdd]=fbKin(X,v,a) calculates the inverse kinematics, of a floating
% base.  (If the first argument is a 6x6 matrix, then it is assumed to be
% X; otherwise, it is assumed to be q.)  The vectors q, qd and qdd are the
% position, velocity and acceleration variables of the first 6 joints in a
% system model created by floatbase; X is the coordinate transform from
% fixed to floating base coordinates; and v and a are the spatial velocity
% and acceleration of the floating base expressed in fixed-base
% coordinates.  In effect, fbKin calculates the forward and inverse
% kinematics of a 6-DoF Cartesian robot.  The returned value of q(5) is
% normalized to the range -pi/2 to pi/2; and the returned values of q(4)
% and q(6) are normalized to the range -pi to pi.  The inverse kinematics
% function fails if the configuration is too close to a singularity.
% Singularities occur at q(5)=+-pi/2.

if all(size(i1)==[6 6])
  [o1, o2, o3] = invkin( i1, i2, i3 );
else
  [o1, o2, o3] = fwdkin( i1, i2, i3 );
end



function  [X, v, a] = fwdkin( q, qd, qdd )

c4 = cos(q(4));  s4 = sin(q(4));
c5 = cos(q(5));  s5 = sin(q(5));
c6 = cos(q(6));  s6 = sin(q(6));

E = [  c5*c6,  c4*s6+s4*s5*c6,  s4*s6-c4*s5*c6;
      -c5*s6,  c4*c6-s4*s5*s6,  s4*c6+c4*s5*s6;
        s5,       -s4*c5,           c4*c5 ];

r = q(1:3);

R = [  0,    -r(3),  r(2);
       r(3),  0,    -r(1);
      -r(2),  r(1),  0 ];

X = [ E, zeros(3); -E*R, E ];

S = [ 1  0    s5;
      0  c4  -s4*c5;
      0  s4   c4*c5 ];

omega = S*qd(4:6);
rd = qd(1:3);

v = [ omega; rd+cross(r,omega) ];

c4d = -s4*qd(4);  s4d = c4*qd(4);
c5d = -s5*qd(5);  s5d = c5*qd(5);

Sd = [ 0  0     s5d;
       0  c4d  -s4d*c5-s4*c5d;
       0  s4d   c4d*c5+c4*c5d ];

omegad = S*qdd(4:6) + Sd*qd(4:6);
rdd = qdd(1:3);

a = [ omegad; rdd+cross(rd,omega)+cross(r,omegad) ];



function  [q, qd, qdd] = invkin( X, v, a )

E = X(1:3,1:3);
rx = -E'*X(4:6,1:3);
r = [ rx(3,2); rx(1,3); rx(2,1) ];

q(1:3,1) = r;

q(5) = atan2( E(3,1), sqrt(E(1,1)*E(1,1)+E(2,1)*E(2,1)) );
q(6) = atan2( -E(2,1), E(1,1) );
if E(3,1) > 0
  q(4) = atan2( E(2,3)+E(1,2), E(2,2)-E(1,3) ) - q(6);
else
  q(4) = atan2( E(2,3)-E(1,2), E(2,2)+E(1,3) ) + q(6);
end
if q(4) > pi
  q(4) = q(4) - 2*pi;
elseif q(4) < -pi
  q(4) = q(4) + 2*pi;
end

c4 = cos(q(4));  s4 = sin(q(4));
c5 = cos(q(5));  s5 = sin(q(5));

S = [ 1  0    s5;
      0  c4  -s4*c5;
      0  s4   c4*c5 ];

omega = v(1:3);
rd = v(4:6) - cross(r,omega);

qd(1:3,1) = rd;
qd(4:6) = S \ omega;

c4d = -s4*qd(4);  s4d = c4*qd(4);
c5d = -s5*qd(5);  s5d = c5*qd(5);

Sd = [ 0  0     s5d;
       0  c4d  -s4d*c5-s4*c5d;
       0  s4d   c4d*c5+c4*c5d ];

omegad = a(1:3);
rdd = a(4:6) - cross(rd,omega) - cross(r,omegad);

qdd(1:3,1) = rdd;
qdd(4:6) = S \ (omegad - Sd*qd(4:6));

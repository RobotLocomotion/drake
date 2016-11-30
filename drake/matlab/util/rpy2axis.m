function a = rpy2axis(rpy)

% converts roll, pitch, yaw notation (e.g., as used in URDF) to axis angle
% notation (e.g. as used by VRML)
% from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToAngle/index.htm

typecheck(rpy,'numeric');
sizecheck(rpy,3);

a = quat2axis(rpy2quat(rpy));


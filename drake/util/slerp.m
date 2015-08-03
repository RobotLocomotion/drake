function Q = slerp(q1, q2, f)
  % Q = slerp(q1, q2, f) Spherical linear interpolation between two quaternions
  %   This function uses the implementation given in Algorithm 8 of [1].
  %
  % @param q1   Initial quaternion (w, x, y, z)
  % @param q2   Final quaternion (w, x, y, z)
  % @param f    Interpolation parameter. N-element row vector of values between
  %             0 and 1 (inclusive)
  % @retval Q   Interpolated quaternion(s). 4-by-N array.
  %
  % [1] Kuffner, J.J., "Effective sampling and distance metrics for 3D rigid
  % body path planning," Robotics and Automation, 2004. Proceedings. ICRA '04.
  % 2004 IEEE International Conference on , vol.4, no., pp.3993,3998 Vol.4,
  % April 26-May 1, 2004
  % doi: 10.1109/ROBOT.2004.1308895

  % Compute the quaternion inner product
  lambda = q1'*q2;

  if lambda < 0
    % The quaternions are pointing in opposite directions, so use the equivalent
    % alternative representation for q2
    q2 = -q2;
    lambda = -lambda;
  end

  % Calculate interpolation factors
  if abs(1-lambda) < eps
    % The quaternions are nearly parallel, so use linear interpolation
    r = 1 - f;
    s = f;
  else
    % Calculate spherical linear interpolation factors
    alpha = acos(lambda);
    gamma = 1./sin(alpha);
    r = sin((1-f).*alpha).*gamma;
    s = sin(f.*alpha).*gamma;
  end

  % Set the interpolated quaternion(s)
  Q = bsxfun(@times, q1, r) + bsxfun(@times, q2, s);
  Q = bsxfun(@rdivide, Q, sqrt(sum(Q.^2,1)));

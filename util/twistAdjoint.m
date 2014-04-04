function ret = twistAdjoint(T)
% returns the adjoint to a twist, i.e. returns
% [omega^ 0;
%  v^     omega^]
%
% for T =
% [omega;
%  v]

omega = T(1 : 3);
v = T(4 : 6);

omega_hat = vectorToSkewSymmetric(omega);
v_hat = vectorToSkewSymmetric(v);

ret = [omega_hat zeros(3, 3);
	     v_hat     omega_hat];
end
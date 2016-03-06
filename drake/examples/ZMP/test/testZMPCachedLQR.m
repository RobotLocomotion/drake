function testZMPCachedLQR()

path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'ZMP'));

h_over_g = 0.84 / 9.81;
Qy = diag([0 0 0 0 1 1]);

[A, B, C, D, Q, R, Q1, R1, N] = LinearInvertedPendulum.setupLinearSystem(h_over_g, Qy);

[K, S] = lqr(A, B, Q1, R1, N);

[K_cache, S_cache] = ZMPCachedLQR(h_over_g, Qy);

valuecheck(K, K_cache, 1e-6);
valuecheck(S, S_cache, 1e-6);

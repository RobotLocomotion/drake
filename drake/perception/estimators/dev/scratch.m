M = reshape(1:9, [3, 3])';
M(3, 3) = 10;

MM = M' * M;

[U, D] = eig(MM);
d = diag(D);
[d, I] = sort(d, 'descend');
D = diag(d);
U = U(:, I);

L = 1 ./ sqrt(d);
if det(M) < 0
    L(3) = -L(3);
end

R = M * U * diag(L) * U'

%%
e = [10, -5, -10, 5]';
E = diag(e);
[ea] = eig(E)'

function decompExample()
% An example script to help clarify the msspoly function decomp(q,v)

checkDependency('spotless');

a = msspoly('a');
b = msspoly('b');
c = msspoly('c');
x = msspoly('x');
y = msspoly('y');
z = msspoly('z');
h = [37*a^2*(x^2*y) a*b*(x^3*y+7*z^2); 3*a^4*(x^2*y) 7*a*b^2*(x^3*y+19*z^2)];
[A, B, C] = decomp(h',[a; b; c]);
% pause();
% [A, B, C] = decomp(h');
display(h);
display(A);
display(full(B));
display(C);

% a_data = [1:4];
% b_data = [1:4];
% c_data = [1:4];
% D = msubs(C(4,3), [a; b; c], [a_data;b_data;c_data]);
% full(D)

% h = [a^2*(x^2*y+2*y) a*b*(x^2*y+z*y^2)]
% [D E F] = decomp(h,[a; b; c])
function test_datan2
% first test some random numbers
x = randn();
y = randn();
test_userfun(x,y);

% test x,y in first quadrant
x = 100*abs(randn());
y = 100*abs(randn());
test_userfun(x,y);

% test x,y in second quadrant
x = -100*abs(randn());
y = 100*abs(randn());
test_userfun(x,y);

% test x,y in third quadrant
x = -100*abs(randn());
y = -100*abs(randn());
test_userfun(x,y);

% test x,y in forth quadrant
x = 100*abs(randn());
y = -100*abs(randn());
test_userfun(x,y);

% test x,y at the border between first and second quadrant
x = 0;
y = 100*abs(randn());
test_userfun(x,y);

% test x,y at the border between second and third quadrant
x = -100*abs(randn());
y = 0;
test_userfun(x,y);

% test x,y at the border between third and forth quadrant
x = 0;
y = -100*abs(randn());
test_userfun(x,y);

% test x,y at the border between forth and first quadrant
x = 100*abs(randn());
y = 0;
test_userfun(x,y);
end

function test_userfun(x,y)
g = datan2(x,y);
[~,g_num] = geval(@(x,y) atan2(y,x),x,y,struct('grad_method','numerical'));
valuecheck(g,g_num,1e-3);
end
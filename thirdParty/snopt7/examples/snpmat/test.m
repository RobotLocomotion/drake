clear;
A  = [ -1 -2 -2;
        1  2  2];
b  = [ 0; 72 ];
x0 = [ 10; 10; 10 ];

%[x] = fmincon(testobj,x0,A,b)
snprint('test.out');

[x,fval] = snpmat(@testobj,x0,A,b)
snprint off;

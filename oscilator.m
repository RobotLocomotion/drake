function dx = oscilator(t, x)
dx = zeros(2, 1);

m  = 1.0;
k = 3947.84;
d = 31.831;
 
Fext = -10;

Fk = -k * x(1) * (1 - d * x(2));

dx(1) = x(2);
dx(2) = (Fk + Fext) / m;

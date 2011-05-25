% test_polygeom.m - test polygeom
%   area, centroid, perimeter and area moments of polygonal outline
% H.J. Sommer III - 02.05.14 - tested under MATLAB v5.2

clear

% constants
d2r = pi / 180;

% 3x5 test rectangle with long axis at 30 degrees
% area=15, x_cen=3.415, y_cen=6.549, perimeter=16
% I1=11.249, I2=31.247, J=42.496
x = [ 2.000  0.500  4.830  6.330 ]';
y = [ 4.000  6.598  9.098  6.500 ]';

% get geometry
[ geom, iner, cpmo ] = polygeom( x, y );

% show results
area = geom(1);
x_cen = geom(2);
y_cen = geom(3);
perimeter = geom(4);
disp( [ ' ' ] )
disp( [ '3x5 test rectangle with long axis at 30 degrees' ] )
disp( [ ' ' ] )
disp( [ '      area     x_cen     y_cen     perim' ] )
disp( [ area  x_cen  y_cen  perimeter ] )

I1 = cpmo(1);
angle1 = cpmo(2);
I2 = cpmo(3);
angle2 = cpmo(4);
disp( [ ' ' ] )
disp( [ '        I1        I2' ] )
disp( [ I1 I2 ] )
disp( [ '    angle1    angle2' ] )
disp( [ angle1/d2r angle2/d2r ] )

% plot outline
xplot = [ x ; x(1) ];
yplot = [ y ; y(1) ];
rad = 10;
x1 = [ x_cen-rad*cos(angle1)  x_cen+rad*cos(angle1) ];
y1 = [ y_cen-rad*sin(angle1)  y_cen+rad*sin(angle1) ];
x2 = [ x_cen-rad*cos(angle2)  x_cen+rad*cos(angle2) ];
y2 = [ y_cen-rad*sin(angle2)  y_cen+rad*sin(angle2) ];
plot( xplot,yplot,'b', x_cen,y_cen,'ro', ...
      x1,y1,'g:', x2,y2,'g:'  )
axis( [ 0  rad  0  rad ] )
axis square


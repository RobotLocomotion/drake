
close all

A=[-sqrt(6)/2,-3*sqrt(2)/2,-sqrt(3)/2;sqrt(6),0,-sqrt(3)/2;-sqrt(6)/2,3*sqrt(2)/2,-sqrt(3)/2;0,0,1];
b=[-sqrt(6)/2;-sqrt(6)/2;-sqrt(6)/2;0];

plotregion(A,b,[],[],[0.1,0.5,0.8]);

axis equal

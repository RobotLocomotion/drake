
close all

A=[-sqrt(6)/2,-3*sqrt(2)/2,-sqrt(3)/2;sqrt(6),0,-sqrt(3)/2;-sqrt(6)/2,3*sqrt(2)/2,-sqrt(3)/2;0,0,1];
b=[-sqrt(6)/2;-sqrt(6)/2;-sqrt(6)/2;0];

lb=[-0.3,-999,0];
ub=[0.3,999,0.9];

plotregion(A,b,lb,ub,[0.1,0.9,0.0]);

axis equal

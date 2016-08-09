
close all

A=[1,1,1];
b=2;
lb=zeros(3,1);
ub=ones(3,1);

plotregion(A,b,lb,ub,[0.3,0.3,0.9]);

axis equal

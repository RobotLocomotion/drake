
close all

n=15;
vec1=sin(linspace(0,2*pi*(1-1/n),n))';
vec2=cos(linspace(0,2*pi*(1-1/n),n))';
vec3=zeros(n,1);

plotregion({-[vec1,vec2,vec3;vec3,vec2,vec1],[1 0 0;-1 0 0;0 1 0;0 -1 0;0 0 1;0 0 -1]*[cos(1),sin(1),0;cos(3),0,sin(3);0,sin(2),cos(2)]},{-1.9*ones(2*n,1),[-5 1 5 -6 -3 2]'},[],[],{'b','r'},0.7);

axis equal

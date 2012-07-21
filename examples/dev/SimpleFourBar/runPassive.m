function runPassive

p = PlanarURDFManipulator('FourBar.urdf');
v = PlanarURDFVisualizer('FourBar.urdf',[-4 4 -3 4]);

x0 = resolveConstraints(p,[pi;pi;.1;0;0;0]);
xtraj = p.simulate([0 5],x0);

draw(v,0,x0+[-.1;-.2;0;0;0;0])
hold on;
plot(0,0,'k.');
axisAnnotation('arrow',-.87+[0;0],-.15+[0;1.75],'LineWidth',2,'Color',MITred,'HeadLength',150,'HeadWidth',150);
text(-1.5,1.2,'$\lambda_1$','interpreter','latex','FontSize',20,'Color',MITred);
axisAnnotation('arrow',-.87+[0;1.75],-.15+[0;0],'LineWidth',2,'Color',MITred,'HeadLength',150,'HeadWidth',150);
text(.7,.2,'$\lambda_2$','interpreter','latex','FontSize',20,'Color',MITred);

v.playback(xtraj);

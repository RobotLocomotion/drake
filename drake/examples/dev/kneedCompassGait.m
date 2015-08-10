function kneedCompassGait(X0)

% KneedCompassGait
%   This function simulates the passive 5-link compass gait model 
%   described in Hsu07
%
% This code was written by Russ Tedrake <russt@mit.edu>.  Please
% acknowlege that fact if you find this code useful.  Feel free to
% email me with questions or comments.

 global g mh ml ms lt ls l a1 a2 b1 b2 gamma;

 g = 9.8;
 lt = .5;
 ls = .5;
 l = ls + lt;
 gamma = asin(7.25/144);

 linkR = [.4;.7];
 massR = [.5;10];
 mt = .5;
mh = 2*massR(1)*mt;
ms = mt/massR(2);

bRightStance = false;
bLocked = 0;

if (nargin < 2) bDraw = true; end


% mR = .5;10
% fP = [0.1877;-0.2884;-0.2884;-1.33;-3.29;-3.29];
% mR = .5;4
% fP = [0.189123463312659;0.289860497120154;-0.289860497120154;-1.114780229929899;-0.257668352979864;-0.257668352979864;];

if (nargin < 1)
%  a = -30*pi/180;
%  q1 = -a/2-gamma;
%  q2 = -gamma+a/2;
%  x0 = [q1;q2;q2;-1.4;-.6;-.6];
 % interleg angle of 0.476137246414833 rad, 27.28065468855042 deg
 x0 = [0.187700106303669;-0.288437140111164;-0.288437140111164;-1.101439041094875;-0.039885255927632;-0.039885255927632;];
end

Q(:,1) = [x0; bRightStance];
LC = x0;
FC = x0;

% Percentage of the link that is below the center of mass (a's)
rt = linkR(1);
rs = linkR(2);

a1 = rs*ls;
b1 = (1-rs)*ls;
a2 = rt*lt;
b2 = (1-rt)*lt;


dt = 1e-3;
displaydt = 0.02;
T = 5;

lastt = 0;
if (bDraw)
   draw(Q(:,1),0);
end
[KE, PE] = mechEnergy(Q(1:6,1),bLocked);
n = 0; % number of steps;
elapsedT = 0;
t = 0;

while (elapsedT<T)
 elapsedT = elapsedT + dt;
 t = t + 1;
 if (bLocked)
   Q(1:6,t+1) = Q(1:6,t) + dt*[Q(4:5,t);Q(5,t);lockedDynamics(Q(1:3,t),Q(4:6,t))];
 else
   Q(1:6,t+1) = Q(1:6,t) + dt*[Q(4:6,t);unlockedDynamics(Q(1:3,t),Q(4:6,t))];
 end
 Q(7,t+1) = bRightStance;
 [KEi,PEi] = mechEnergy(Q(1:6,t+1),bLocked);
 KE = [KE ; KEi];
 PE = [PE ; PEi];

 % knee collision and lock
 if (Q(3,t+1)>Q(2,t+1))
   Qadd = [];
   t = collisionSearch(Q(1:6,t),dt,t,0);
   Q = [Q Qadd];
   a = (Q(1,t+1) - Q(2,t+1));
   b = (Q(1,t+1) - Q(3,t+1));
   c = (Q(2,t+1) - Q(3,t+1));
   Qm11 = (ms*a1^2+mt*a2^2+(-mt*l*b2-ms*l*lt)*cos(a)+mh*l^2+ms*l^2-ms*b1*l*cos(b)+2*mt*a2*ls+mt*l^2+mt*ls^2);
   Qm12 = ((-mt*l*b2-ms*l*lt)*cos(a)+mt*b2^2+ms*b1*lt*cos(c)+ms*lt^2);
   Qm13 = (-ms*b1*l*cos(b)+ms*b1^2+ms*b1*lt*cos(c));
   Qm21 = ((-mt*l*b2-ms*l*lt)*cos(a)-ms*b1*l*cos(b));
   Qm22 = (ms*b1*lt*cos(c)+ms*lt^2+mt*b2^2);
   Qm23 = (ms*b1*lt*cos(c)+ms*b1^2);
   Qminus = [Qm11 Qm12 Qm13; Qm21 Qm22 Qm23];
   Qp11 = (-ms*l*lt-mt*l*b2-ms*b1*l)*cos(a)+ms*a1^2+ms*l^2+2*mt*a2*ls+mt*a2^2+mt*ls^2+mt*l^2+mh*l^2;
   Qp12 = (-ms*l*lt-mt*l*b2-ms*b1*l)*cos(a)+ms*lt^2+2*ms*b1*lt+ms*b1^2+mt*b2^2;
   Qp21 = (-ms*l*lt-mt*l*b2-ms*b1*l)*cos(a);
   Qp22 = ms*b1^2+2*ms*b1*lt+ms*lt^2+mt*b2^2;
   Qplus = [Qp11 Qp12 ; Qp21 Qp22];
   Q(4:5,t+1) = inv(Qplus)*(Qminus*Q(4:6,t+1));
   Q(6,t+1) = Q(5,t+1);
   Q(3,t+1) = Q(2,t+1);
   bLocked = 1;
 end

 % heelstrike
 if ((Q(3,t)==Q(2,t)) && ((Q(2,t+1)-Q(1,t+1))>0) &&  ((Q(1,t+1)+Q(2,t+1)) <= -2*gamma))
   Qadd = [];
   t = collisionSearch(Q(1:6,t),dt,t,1);
   Q = [Q Qadd(:,1:end-1)]; % get rid of the last point
   t = t-1; % get rid of the last point
   n = n+1;
   bRightStance = ~bRightStance;
   bLocked = 0;
   a = (Q(2,t+1) - Q(1,t+1));
   Qminus = [(-(ms*a1*(lt+b1)+mt*b2*(ls+a2)) + cos(a)*l*(mh*l + (2*(mt*(a2+ls) + ms*a1)))) (-(ms*a1*(lt+b1)+mt*b2*(ls+a2)));
     -(ms*a1*(b1+lt)+mt*b2*(ls+a2)) 0];
   Qplus = [((-l*(ms*(b1+lt)+mt*b2)*cos(a))+l^2*(ms+mt+mh)+ms*a1^2+mt*(a2+ls)^2) ((-l*(ms*(b1+lt)+mt*b2)*cos(a))+ms*(b1+lt)^2+mt*b2^2);
     (-l*(ms*(b1+lt)+mt*b2)*cos(a)) (mt*b2^2+ms*(b1+lt)^2)];
   Q(4:5,t+1) = inv(Qplus)*Qminus*Q(4:5,t+1);
   Q(6,t+1) = Q(5,t+1);
   % switch stance and swing legs:
   Q(1:3,t+1) = [0 1 ; 1 0 ; 1 0]*Q(1:2,t+1);
   Q(7,t+1) = bRightStance;
   LC = FC;
   FC = Q(1:6,t+1);
   xn = [FC(1)-FC(2);FC(4);FC(5)];
   xn1 = [LC(1)-LC(2);LC(4);LC(5)];
   E = 0.5*(xn1-xn)'*(xn1-xn);
   if (((E<1e-5) && (elapsedT > 10)) || (n>200))
     stable = 1;
     break;
   end
 end

 swing_foot = l*[-sin(Q(1,t+1));cos(Q(1,t+1))] - lt*[-sin(Q(2,t+1));cos(Q(2,t+1))] - ls*[-sin(Q(3,t+1));cos(Q(3,t+1))];
 ramp_y = -swing_foot(1)*tan(gamma);

 % if fall or heel strikes before kneestrike
 if ((cos(Q(1,t+1)) < -gamma*sin(Q(1,t+1))) || ...
     (~(Q(3,t+1)==Q(2,t+1)) && (Q(4,t+1)<0) && (Q(5,t+1)<0) && ((Q(2,t+1)-Q(1,t+1))>0) &&((Q(1,t+1)+Q(2,t+1)) <= -2*gamma)) || ...
     (swing_foot(2) < ramp_y - 0.01))
   break;
 end

 if ((t - lastt > displaydt/dt) && (bDraw))
   figure(10);
   hold on;
   draw(Q(:,t),elapsedT);
   lastt = t;
 end

end

if (bDraw)
 draw(Q(:,end),elapsedT);
end

if 0
   qr = (1-Q(7,:)).*Q(1,:)+Q(7,:).*Q(2,:);
   qrdot = (1-Q(7,:)).*Q(4,:)+Q(7,:).*Q(5,:);
   ql = Q(7,:).*Q(1,:)+(1-Q(7,:)).*Q(2,:);
   qldot = Q(7,:).*Q(4,:)+(1-Q(7,:)).*Q(5,:);

   figure(1); clf;
   plot(qr,qrdot,'b',qr(1),qrdot(1),'k.');
   title('Right Leg'); xlabel('\theta'); ylabel('d\theta/dt');

   figure(2); clf;
   hold on;
   plot(PE,'g');
   plot(KE,'c');
   plot(PE+KE);
   title('Mechanical Energy of the System'); xlabel('time step');
   legend('PE wrt stance foot','KE of system','Total ME');
   hold off;
end


 function t = collisionSearch(q,dt,t,heelstrike)
   dt1 = dt/2;
   if (heelstrike==1)
     qnew = q(1:6) + dt1*[q(4:5);q(5);lockedDynamics(q(1:3),q(4:6))];
     test = ((qnew(1)+qnew(2)) > -2*gamma);
   else %kneestrike
     qnew = q(1:6) + dt1*[q(4:6);unlockedDynamics(q(1:3),q(4:6))];
     test = (qnew(3)<=qnew(2));
   end
   % [KE(t+1),PE(t+1)] = mechEnergy(Q(1:6,t+1));
   qnew(7) = bRightStance;

   if (dt>1e-20) %precision for collision detection
     if (test) %then collision is in second half, so advance dt/2 in time
       t = t+1;
       elapsedT = elapsedT + dt1;
       Qadd = [Qadd qnew];
       q = qnew;
     end
     t = collisionSearch(q,dt1,t,heelstrike);
   end
 end

 function [KE, PE] = mechEnergy(q,locked)
   yh = l*cos(q(1));
   y1 = a1*cos(q(1));
   y2 = (ls+a2)*cos(q(1));
   y3 = l*cos(q(1))-b2*cos(q(2));
   y4 = l*cos(q(1))-lt*cos(q(2))-b1*cos(q(3));
   H1 = ms*[(a1^2) 0 0 ; 0 0 0 ; 0 0 0];
   H2 = mt*[((ls+a2)^2) 0 0 ; 0 0 0 ; 0 0 0];
   Hh = mh*[(l^2) 0 0 ; 0 0 0 ; 0 0 0 ];
   H3 = mt*[(l^2)                  (-l*b2*cos(q(2)-q(1))) 0 ;
     (-l*b2*cos(q(2)-q(1))) (b2^2)                 0 ; 0 0 0 ];
   H4 = ms*[(l^2) (-l*lt*cos(q(2)-q(1))) (-l*b1*cos(q(3)-q(1))) ;
     (-l*lt*cos(q(2)-q(1))) (lt^2) (lt*b1*cos(q(3)-q(2))) ;
     (-l*b1*cos(q(3)-q(1))) (lt*b1*cos(q(3)-q(2))) (b1^2)];
   H = H1+H2+H3+H4+Hh;
   KE = .5*q(4:6)'*H*q(4:6);
   PE = g*(ms*(y1+y4)+mt*(y2+y3)+mh*yh);
 end

 % 2-link configuration
 function qddot = lockedDynamics(q,qdot)
   q1 = q(1); q2 = q(2);

   H11 = (mt+ms+mh)*l^2+ms*a1^2+mt*(a2+ls)^2;
   H12 = (-ms*l*lt+l*(-mt*b2-ms*b1))*cos(q1-q2);
   H21 = H12;
   H22 = mt*b2^2+ms*lt^2+2*ms*lt*b1+ms*b1^2;
   H = [H11 H12; H21 H22];

   h = -l*sin(q1-q2)*(mt*b2+ms*lt+ms*b1);
   C = [0 h ; -h 0] ;
   G = [-g*sin(q1)*(mh*l+ms*a1+ms*l+mt*a2+mt*ls+mt*l);
     g*sin(q2)*(mt*b2+ms*lt+ms*b1)];
   qddot = inv(H)*(-C*qdot(1:2).^2-G);
   qddot = [qddot ; qddot(2)];
 end

 % 3-link configuration
 function qddot = unlockedDynamics(q,qdot)
   q1 = q(1); q2 = q(2); q3 = q(3);
   H(1,1)=(ms*a1^2+mt*(a2+ls)^2+mt*l^2+ms*l^2+mh*l^2);
   H(2,2)= mt*b2^2+ms*lt^2;
   H(3,3)= ms*b1^2;
   H(1,2)= -mt*l*b2*cos(q1-q2)-ms*l*lt*cos(q1-q2);
   H(2,1)= -mt*l*b2*cos(q1-q2)-ms*l*lt*cos(q1-q2);
   H(1,3)= -ms*l*b1*cos(q1-q3);
   H(3,1)= -ms*l*b1*cos(q1-q3);
   H(2,3)= ms*lt*b1*cos(q2-q3);
   H(3,2)= ms*lt*b1*cos(q2-q3);

   h12 = -mt*l*b2*sin(q1-q2)-ms*l*lt*sin(q1-q2);
   h13 = -ms*l*b1*sin(q1-q3);
   h21 = -h12;
   h23 = ms*lt*b1*sin(q2-q3);
   h31 = -h13;
   h32 = -h23;
   C = [0 h12 h13; h21 0 h23; h31 h32 0];

   G =  [ -g*((mh+ms+mt)*l+ms*a1+mt*(a2+ls))*sin(q1);
     g*(ms*lt+mt*b2)*sin(q2);
     g*ms*b1*sin(q3)];
   qddot = inv(H)*(-C*qdot.^2-G);
 end

 function draw(q,t)
   % draw four link walker with 5 point masses
   h = figure(10);
   set(h,'DoubleBuffer','on');
   clf;
   hold on;

   hip = l*[-sin(q(1));cos(q(1))];
   stk = ls*[-sin(q(1));cos(q(1))];
   swk = hip - lt*[-sin(q(2));cos(q(2))];
   swf = swk - ls*[-sin(q(3));cos(q(3))];

   ss = a1*[-sin(q(1));cos(q(1))];
   ts = (ls+a2)*[-sin(q(1));cos(q(1))];
   tns = hip - b2*[-sin(q(2));cos(q(2))];
   sns = swk - b1*[-sin(q(3));cos(q(3))];

   mtd = 0.04;
   mr_sh = (ms/mt)^(1/3);
   mr_h = (mh/mt)^(1/3);

   x = hip(1) + [-1,1];

   line(x, -tan(gamma)*x,'Color',[0 1 0]);

   % keyboard;
   line([0, hip(1)], [0, hip(2)],'LineWidth',2.0,'Color',[(1-q(7)) 0 q(7)]);
   line([hip(1), swk(1)], [hip(2), swk(2)],'LineWidth',2.0,'Color',[q(7) 0 1-q(7)]);
   line([swk(1), swf(1)], [swk(2), swf(2)],'LineWidth',2.0,'Color',[q(7) 0 1-q(7)]);

   x = 0:0.1:2*pi;
   % masses
   fill(hip(1) + mr_h*mtd*sin(x), hip(2) + mr_h*mtd*cos(x),[0 1 0]);
   fill(ss(1) + mr_sh*mtd*sin(x), ss(2) + mr_sh*mtd*cos(x),[1-q(7) 0 q(7)]);
   fill(ts(1) + mtd*sin(x), ts(2) + mtd*cos(x),[1-q(7) 0 q(7)]);
   fill(tns(1) + mtd*sin(x), tns(2) + mtd*cos(x),[q(7) 0 1-q(7)]);
   fill(sns(1) + mr_sh*mtd*sin(x), sns(2) + mr_sh*mtd*cos(x),[q(7) 0 1-q(7)]);

   fill(swk(1) + mr_sh/2*mtd*sin(x), swk(2) + mr_sh/2*mtd*cos(x),[q(7) 0 1-q(7)]);
   fill(stk(1) + mr_sh/2*mtd*sin(x), stk(2) + mr_sh/2*mtd*cos(x),[1-q(7) 0 q(7)]);
   axis equal;
%   axis off;
   hold off;
   title(['t = ', num2str(t)]);
   drawnow;
 end

end
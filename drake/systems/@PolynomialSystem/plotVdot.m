function plotVdot(sys,Vtraj,ts,options)

if (sys.getNumStates()~=2) error('only support 2d systems so far'); end
if (~sys.isCT()) error('only support CT systems so far'); end

num_u = sys.getNumInputs();
p_t=msspoly('t',1);
p_x = sys.p_x;
u = zeros(num_u,1);
r = msspoly('r',1);

Ncontours=4;

figure(1); clf; set(gcf,'DoubleBuffer','on'); hold on;

% for surf:
%[X1,X2] = ndgrid(linspace(-5,5,21),linspace(-5,5,21));
%VdotSurf=X1;

%mov = VideoWriter('Vdot','Motion JPEG AVI');
%open(mov);
      

for t=ts;

%% plot surf+contour by sampling
%  for i=1:prod(size(X1))
%    xs = [X1(i);X2(i)];
%    xdots = sys.dynamics(t,xs,u);
%    VdotSurf(i) = 2*xs'*V.eval(t)*xdots + xs'*V.deriv(t)*xs;
%  end

  clf; 
%  surf(X1,X2,VdotSurf); colorbar;
%  view(0,90);
%  contour(X1,X2,VdotSurf,[0,0],'Color',[1 1 1],'LineWidth',4);
%  drawnow;

%% plot Vdot=0 contour by poly roots  
  xdot=sys.p_dynamics_traj.eval(t);
  V = Vtraj.eval(t);
  if (num_u)
    xdot = subs(xdot,sys.p_u,u);
  end
  Vdot = subs(diff(V,p_x)*xdot + diff(V,p_t),p_t,t);
  [theta,radius]=fplot(@getRoots,[0 pi]);
  y=[];
  for i=1:Ncontours
    y=[y;repmat(radius(:,i),1,2).*[cos(theta),sin(theta)]];
  end
  plot(y(:,1),y(:,2),'b.');
%  axis([-2,2,-2,2]);
  title(['t = ',num2str(t)]);

%  writeVideo(mov,getframe(gcf));

  drawnow;
%  pause;
end

%      close(mov);




function y=getRoots(theta) 
  circ = [cos(theta);sin(theta)];
  for i=1:length(theta)
    [a,p,M] = decomp(subss(Vdot,p_x,r*circ(:,i)));
    c(max(p)+1-p)=M;
    z=roots(c);  % roots of poly V along the line defined by th(i)
    z=z(find(~imag(z)));  % only keep the real roots
    if (length(z)>Ncontours) % keep the ones closest to zero
      [a,i]=sort(abs(z));
      y(i,:)=z(i(1:Ncountours));
    else
      y(i,:)=[z;repmat(nan,Ncontours-length(z),1)];
    end
  end
end

end

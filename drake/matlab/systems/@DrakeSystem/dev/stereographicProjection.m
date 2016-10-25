function polysys = stereographicProjection(sys,x0,options)

% Generates a polynomial system from a trig/poly system using the
% stereographic projection 
% See robotlib/doc/stereographic.pdf

if (nargin<2)
  x0 = zeros(sys.num_x,0);
end

if (nargin<3)
  options=struct();
end
% no options yet, but i'm sure they're coming!

checkDependency('spotless');

t=msspoly('t',1);
theta=msspoly('h',sys.num_x);
s=msspoly('s',sys.num_x);
c=msspoly('c',sys.num_x);
q=TrigPoly(theta,s,c);
x=msspoly('x',sys.num_x);
u=msspoly('u',sys.num_u);

p_mass_matrix=msspoly(eye(sys.num_xc));
if (sys.num_xc>0)
  tp_dynamics = sys.dynamics(t,q,u);
  
  for i=1:sys.num_xc
    p_dynamics(i)=getmsspoly(tp_dynamics(i));
    for j=1:sys.num_xc
      d = deg(subs(p_dynamics(i),c(j),s(j)),s(j)); % get total degree in s and c (combined)
      p_mass_matrix(i,i)=p_mass_matrix*(1+x(j)^2)^d;

      % now do the careful substitution (each degree is a bit different)
      [R,p]=pdecomp(p_dynamics(i),s(j));
      p_dynamics(i)=msspoly(0);
      for k=1:length(p)  % have to unroll this since .^ (power) is not implemented for msspoly
        p_dynamics(i)=p_dynamics(i)+R(k)*(2*x(j))^p(k)*(1+x(j)^2)^(d-p(k));
      end
      
      [R,p]=pdecomp(p_dynamics(i),c(j));
      p_dynamics(i)=msspoly(0);
      for k=1:length(p)  % have to unroll this since .^ (power) is not implemented for msspoly
        p_dynamics(i)=p_dynamics(i)+R(k)*(1-x(j)^2)^p(k)*(1+x(j)^2)^(d-p(k));
      end
    end
  end
else
  p_dynamics=[];
end

%todo:  premultiply by (1+q^2)/2 for derivatives on the altered vars.

function I = inertiaCalculator(type,mass,varargin)
% Usage
%  inertiaCalculator('box',mass,size)
%  inertiaCalculator('sphere',mass,radius)
%  inertiaCalculator('cylinder',mass,radius,length)
% see e.g. http://en.wikipedia.org/wiki/List_of_moments_of_inertia

switch(type)
  case 'box'
    size=varargin{1};
    I = 1/12*mass*diag([size(2)^2+size(3)^2,size(1)^2+size(3)^2,size(1)^2+size(2)^2]);
  case 'sphere'
    radius=varargin{1};
    I = eye(3)*2*mass*radius^2/5;
  case 'cylinder'
    radius=varargin{1};
    length=varargin{2};
    ixx = 1/12*mass*(3*radius^2 + length^2);
    I = diag([ixx,ixx,mass*radius^2/2]);
end

if nargout<1
  fprintf('ixx="%f" ixy="%f" ixz="%f" iyy="%f" iyz="%f" izz="%f"\n',I(1,1),I(1,2),I(1,3),I(2,2),I(2,3),I(3,3));
end
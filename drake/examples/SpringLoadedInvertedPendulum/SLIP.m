classdef SLIP < HybridDrakeSystem

% spring loaded inverted pendulum, control input is the angle of attack

properties
  m=80;  % mass (kg)
  r0=1; % rest length of leg spring (m)
  k=14000; % spring coefficient
  g=9.8; % gravity
  stopAtApex = 0; 
end

methods
    function obj=SLIP()
      obj = obj@HybridDrakeSystem(1,8);
      obj = setInputFrame(obj,CoordinateFrame('SLIPInput',1,'u',{'angle_of_attack'}));
      obj = setOutputFrame(obj,CoordinateFrame('SLIPOutput',8,'y',{'x','y','r','theta','xdot','ydot','rdot','thetadot'}));
      
      pFlight1=SLIPFlight(obj);
      obj=obj.addMode(pFlight1);
      pStance=SLIPStance(obj);
      obj=obj.addMode(pStance);
      pFlight2=SLIPFlight(obj);
      obj=obj.addMode(pFlight2);
      
      obj=addTransition(obj,1,@collisionGuard,@flight2stance,true,true);
      obj=addTransition(obj,3,@apexGuard,@reachApex,false,true);
      obj=addTransition(obj,2,andGuards(obj,@takeOffGuard1,@takeOffGuard2),@stance2flight,false,true);
    end
    
    function [g,dg]=collisionGuard(obj,t,x,u)
      g=x(2)-obj.r0*cos(u);  % foot hits the ground
      dg=[0 0 1 0 0 obj.r0*sin(u)];
    end
    
    function [g,dg]=takeOffGuard1(obj,t,x,u)
      g=obj.r0-x(1);  % r >= r0
      dg=[0 -1 0 0 0 0];
    end
    
    function [g,dg]=takeOffGuard2(obj,t,x,u)
      g=-x(3);  % rdot >= 0
      dg=[0 0 0 -1 0 0];
    end
    
    function [g,dg]=apexGuard(obj,t,x,u)
      g=x(4);  % ydot <= 0
      dg=[0 0 0 0 1 0];
    end
    
    function [xp,mode,status,dxp]=stance2flight(obj,mode,t,xm,u)
      xp=[xm(5)-xm(1)*sin(xm(2));...
        xm(1)*cos(xm(2));...
        -xm(3)*sin(xm(2))-xm(1)*xm(4)*cos(xm(2));...
        xm(3)*cos(xm(2))-xm(1)*xm(4)*sin(xm(2));];
      if(mode~=2)
        error('Incorrect mode');
      else
        mode=3;
      end
      status=(xp(3)<0);  % terminate if xdot < 0
      dxpdxm=zeros(4,5);
      dxpdxm(1,5)=1;
      dxpdxm(1,1)=-sin(xm(2));
      dxpdxm(1,2)=-xm(1)*cos(xm(2));
      dxpdxm(1,5)=1;
      dxpdxm(2,1)=cos(xm(2));
      dxpdxm(2,2)=-xm(1)*sin(xm(2));
      dxpdxm(2,6)=1;
      dxpdxm(3,1)=-xm(4)*cos(xm(2));
      dxpdxm(3,2)=-xm(3)*cos(xm(2))+xm(1)*xm(4)*sin(xm(2));
      dxpdxm(3,3)=-sin(xm(2));
      dxpdxm(3,4)=-xm(1)*cos(xm(2));
      dxpdxm(4,1)=-xm(4)*sin(xm(2));
      dxpdxm(4,2)=-xm(3)*sin(xm(2))-xm(1)*xm(4)*cos(xm(2));
      dxpdxm(4,3)=cos(xm(2));
      dxpdxm(4,4)=-xm(1)*sin(xm(2));
      dxp=[zeros(4,2) dxpdxm zeros(4,1)];
    end
    
    function [xp,mode,status,dxp]=flight2stance(obj,mode,t,xm,u)
      if(mode~=1)
        error('Incorrect mode');
      else
        mode=2;
      end
      theta=u;
      r= xm(2)/cos(theta); % = obj.r0 because x(2)-obj.r0*cos(u) = 0
      xp=[r;...
        theta;...
        -xm(3)*sin(theta)+xm(4)*cos(theta);...
        -(xm(3)*cos(theta)+xm(4)*sin(theta))/r;...
        xm(1)+r*sin(theta)]; %xm(2)*tan(theta)];
      status=0;
      if nargout>3
        dxpdxm=zeros(5,4);
        dxpdu=zeros(5,1);
        dxpdxm(1,2)=1/cos(theta);
        dxpdu(1,1)=(xm(2))*sin(theta)/cos(theta)^2;
        %dxpdxm(1,6)=-1/cos(theta);
        dxpdu(2,1)=1;
        dxpdxm(3,3)=-sin(theta);
        dxpdxm(3,4)=cos(theta);
        dxpdu(3,1)=-xm(3)*cos(theta)-xm(4)*sin(theta);
        dxpdxm(4,2)=(xm(3)*cos(theta)^2+xm(4)*sin(theta)*cos(theta))/(xm(2))^2;
        dxpdxm(4,3)=-cos(theta)^2/(xm(2));
        dxpdxm(4,4)=-sin(theta)*cos(theta)/(xm(2));
        dxpdu(4,1)=-(-xm(3)*sin(2*theta)+xm(4)*cos(2*theta))/(xm(2));
        %dxpdxm(4,6)=-dxpdxm(4,2);
        dxpdxm(5,1)=1;
        dxpdxm(5,2)=tan(theta);
        error('still need to update this last one:');
        dxpdu(5,1)=xm(2)/cos(theta)^2;
        dxp=[zeros(5,2) dxpdxm dxpdu];
      end
    end
    
    function [xp,mode,status,dxp]=reachApex(obj,mode,t,xm,u)
      if(mode~=3)
        error('Incorrect mode');
      else
        mode=1;
      end
      xp=xm;
      if(xp(2)<obj.r0*cos(u))
        error('toe touches ground at apex')
      end
      status=obj.stopAtApex;
      dxp=[zeros(4,2) eye(4),zeros(4,1)];
    end
    
    %     function xp=flight2stance(obj,x,theta)
    %         if(x(1)~=1)
    %             error('The first entry must be mode, and in stance mode');
    %         end
    %         xc=x(2:5);
    %         r=xc(2)/cos(theta);
    %         xcn=[r;theta;...
    %             -xc(3)*sin(theta)+xc(4)*cos(theta);...
    %             -(xc(3)*cos(theta)+xc(4)*sin(theta))/r];
    %         xp=[2;xcn];
    %     end
    
    function yn=apex2apex(obj,theta,y0,xdot0)
      if(nargin==2)
        x0=obj.getInitialState;
      else
        x0=[0;y0;xdot0;0];
      end
      obj.stopAtApex=1;
      yTraj=simulate(cascade(setOutputFrame(ConstantTrajectory(theta),getInputFrame(obj)),obj),[0 10],[x0;0]);
      yApex=Point(getOutputFrame(obj),yTraj.eval(yTraj.tspan(end)));
      if yApex.xdot<0
        yn = nan;
      else
        yn = yApex.y;
      end
    end
    
    function theta=angleOfAttackControl(obj,yd,y0,xdot0)
      aoaControl=@(theta) obj.apex2apex(theta,y0,xdot0)-yd;
      theta=fzero(aoaControl,pi/8);
    end
  end
  
  methods (Static)
    function run()
      r = SLIP();
      v = SLIPVisualizer(r);
      
      aoa_command = setOutputFrame(ConstantTrajectory(.5),getInputFrame(r));
      ytraj = simulate(cascade(aoa_command,r),[0 5]);
      v.playback(ytraj);
    end
  end
end
classdef UnderwaterAcrobotPlant < Manipulator
    % Submerged Acrobot, assumes ellipse-shaped links
    properties
        g = 9.81;
        rho = 1000;
        
        semiX1 = 0.5;%Ellipse x semiaxes, link 1
        semiY1 = 1; %Ellipse y semiaxes, link 1
        semiX2 = 0.5;%Ellipse x semiaxes, link 2
        semiY2 = 1; %Ellipse y semiaxes, link 2
        mr1 = .5; %Mass ratio link1 (rholink/rhowater)
        mr2 = .5; %Mass ratio link2 (rholink/rhowater)
        
        %Setting drag to zero for validation against URDF implementation
        cd = 0; %Drag coefficient for bluff body (assume same in x and y)
        
        %Link 1 derived parameters - sorry, these are big because water is heavy
        L1=2; lc1=1; m1=800; I1=250; mx1=3000; my1=800; mth1=200;
        
        %Link 2 derived parameters
        L2=2; lc2=1; m2=800; I2=250; mx2=3000; my2=800; mth2=200;
        
        xG;
        uG;
    end
    
    methods
        function obj = UnderwaterAcrobotPlant
            obj = obj@Manipulator(2,1);
            %obj = setInputLimits(obj,-10,10);
            
            obj = setInputFrame(obj,CoordinateFrame('UnderwaterAcrobotInput',1,'u',{'tau'}));
            obj = setStateFrame(obj,CoordinateFrame('UnderwaterAcrobotState',4,'x',{'theta1','theta2','theta1dot','theta2dot'}));
            obj = setOutputFrame(obj,obj.getStateFrame);
            
            
            obj = setParamFrame(obj,CoordinateFrame('UnderwaterAcrobotParams',9,'p',...
                { 'g','rho','semiX1','semiY1','semiX2','semiY2','mr1','mr2','cd'}));
            obj = setParamLimits(obj,zeros(obj.getParamFrame.dim,1));
            
            %Link 1 parameters
            %[obj.L1,obj.m1,obj.lc1,obj.I1,obj.mx1,obj.my1,obj.mth1] = deriveLinkParams(obj.semiX1,obj.semiY1,obj.mr1,obj.rho);
            %Link 2 parameters
            %[obj.L2,obj.m2,obj.lc2,obj.I2,obj.mx2,obj.my2,obj.mth2] = deriveLinkParams(obj.semiX2,obj.semiY2,obj.mr2,obj.rho);
            
            %Goal position should be unstable eq point
            [~,xUnstable] = eqPoints(obj);
            
            obj.xG = Point(obj.getStateFrame,xUnstable);
            obj.uG = Point(obj.getInputFrame,0);
            
        end
        function [L,m,lc,I,mx,my,mth] = deriveLinkParams(a,b,mr,rho)
            %Sets derived parameters, helper function for constructor
            %Mass parameters
            L = b*2;
            lc = b;
            m = pi*a*b*mr*rho;
            I = 1/4*m*(a^2+b^2);
            
            %Added mass coefficients
            mx = pi*b*b*rho;
            my = pi*a*a*rho;
            mth = 1/8*pi*rho*(a^2-b^2)^2;
        end
        function [xStable,xUnstable] = eqPoints(obj)
            %Helper function for constructor, sets stable and unstable eqs
            %based off potential energy
            g1 = obj.g*(1-1/obj.mr1);
            g2 = obj.g*(1-1/obj.mr2);
            
            %Candidate equilibrium points
            th1 = [0;0;pi;pi];
            th2 = [0;pi;0;pi];
            PE = zeros(4,1); %Potential energy
            for i = 1:4
                y1cm = -obj.lc1*cos(th1(i));
                y2cm = -obj.L1*cos(th1(i))-obj.lc2*cos(th1(i)+th2(i));
                PE(i) = obj.m1*g1*y1cm+obj.m2*g2*y2cm;
            end
            [~,stableIndex] = min(PE);
            [~,unstableIndex] = max(PE);
            
            xStable = [th1(stableIndex);th2(stableIndex);0;0];
            xUnstable = [th1(unstableIndex);th2(unstableIndex);0;0];
        end
        function [H,C,B] = manipulatorDynamics(obj,q,qd)
            %Determines manipulator equation for a submerged Acrobot
            
            %Unpack link 1
            m1=obj.m1; mx1=obj.mx1; my1 = obj.my1; I1 = obj.I1; mth1 = obj.mth1; lc1=obj.lc1; L1 = obj.L1;
            semiX1 = obj.semiX1; mr1=obj.mr1;
            %Unpack link 2
            m2=obj.m2; mx2=obj.mx2; my2 = obj.my2; I2 = obj.I2; mth2 = obj.mth2; lc2=obj.lc2;
            semiX2 = obj.semiX2; semiY2 = obj.semiY2; mr2=obj.mr2;
            
            g=obj.g; cd = obj.cd; rho = obj.rho;
            
            th1 = q(1,:); th2 = q(2,:);
            w1 = qd(1,:); w2 = qd(2,:);
            s1 = sin(th1); s2 = sin(th2); c2 = cos(th2);
            s12 = sin(th1+th2);
            
            %Drag terms (simplier to see)
            v1x = w1*lc1;
            v2x = L1*w1.*c2+lc2*(w1+w2);
            v2y = -L1*w1.*s2;
            Dx1 = rho*semiX1*v1x.*abs(v1x)*cd; %Drag force in link1 x body frame
            Dx2 = rho*semiX2*v2x.*abs(v2x)*cd; %Drag force in link2 x body frame
            Dy2 = rho*semiY2*v2y.*abs(v2y)*cd; %Drag force in link2 y body frame
            
            %% Equations of Motion (from Lagrange derivation)
            % Inertia/added mass
            H11 = (m1+mx1)*lc1^2+(I1+mth1)+(m2+mx2)*(L1^2*c2.^2+lc2.^2+2*L1*lc2*c2)+(m2+my2)*L1^2*s2.^2 + (I2+mth2);
            H12 = (m2+mx2)*(lc2^2+L1*lc2*c2)+I2+mth2;
            H22 = (m2+mx2)*lc2.^2+I2+mth2+zeros(1,length(th1));
            H21 = H12;
            % Coriolis and added mas coriolis
            C1 = (m2+mx2)*(-2*L1^2*w1.*w2.*c2.*s2-L1*lc2*s2.*(w2.^2+2*w1.*w2))+(m2+my2)*(2*L1^2*w1.*w2.*c2.*s2);
            C2 = (m2+mx2)*(L1^2*w1.^2.*c2.*s2+L1*lc2*w1.^2.*s2)+(m2+my2)*-L1^2*w1.^2.*s2.*c2;
            % Gravity torques
            G1 = lc1*s1*m1*g*(1-1/mr1)+L1*s1*m2*g*(1-1/mr2)+lc2*s12*m2*g*(1-1/mr2);
            G2 = lc2*m2*g*s12*(1-1/mr2);
            % Drag forces
            D1 = lc1*Dx1+(L1*c2+lc2).*Dx2-L1*s2.*Dy2; % Drag moments from link 1 and link 2 on base
            D2 = lc2*Dx2; % Drag moment from link 2 on joint
            
            H = [H11 H12; H21 H22];
            C = [C1; C2]+[G1; G2]+[D1;D2];
            
            B = [0; 1];
            
            
        end
        
        function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
            f = dynamics@Manipulator(obj,t,x,u);
            if (nargout>1)
                df = dynamicsGradients(obj,t,x,u,1);
                if (nargout>2)
                    % Analytic gradients did not solve for n>2, so using geval
                    [~,~,d2f,d3f] = geval('dynamics',obj,t,x,u,struct('grad_method','taylorvar'));
                end
            end
        end
        
        function c =balanceLQR(obj)
            Q = diag([10,10,1,1]); R = 1e-12; %Small R because masses are huge
            c = tilqr(obj,obj.xG,obj.uG,Q,R);
        end
        
        function [utraj,xtraj]=swingUpTrajectory(obj)
            [x0,~]=eqPoints(obj); tf0 = 4; xf = double(obj.xG);
            
            con.u.lb = obj.umin;
            con.u.ub = obj.umax;
            con.x0.lb = x0;
            con.x0.ub = x0;
            con.xf.lb = xf;
            con.xf.ub = xf;
            con.T.lb = 2;
            con.T.ub = 6;
            
            options.method='dircol';
            %options.grad_test = true;
            info=0;
            while (info~=1)
                utraj0 = PPTrajectory(foh(linspace(0,tf0,21),randn(1,21)));
                tic
                [utraj,xtraj,info] = trajectoryOptimization(obj,@cost,@finalcost,x0,utraj0,con,options);
                toc
            end
            
            function [g,dg] = cost(t,x,u)
                R = 1;
                g = sum((R*u).*u,1);
                dg = [zeros(1,1+size(x,1)),2*u'*R];
            end
            
            function [h,dh] = finalcost(t,x)
                h = t;
                dh = [1,zeros(1,size(x,1))];
            end
            
        end
    end
    
end

classdef UnderwaterAcrobotPlant < Manipulator
    % Submerged Acrobot, assumes ellipse-shaped links
    properties
        g = 9.81; %Should match RigidBodyManipulator
        rho = 1000;
        
        L1 = 1; %link1 length
        L2 = 1; %link2 length
        d1 = .1; %link 1 diameter
        d2 = .1; %link 2 diameter
        mr1 = .5; %Mass ratio link1 (rholink/rhowater)
        mr2 = .5; %Mass ratio link2 (rholink/rhowater)
        b1 = 0.1;  %Shoulder joint viscous damping
        b2 = 0.1;  %Elbow joint viscous damping
        
        %Setting drag to zero for validation against URDF implementation
        cd = 0; %Drag coefficient for bluff body (assume same in x and y)
        
        %Link 1 derived parameters
        %m1; lc1; I1; mx1; my1; mth1;
        
        %Link 2 derived parameters
        %m2; lc2; I2; mx2; my2; mth2;
        
        %Values in URDF file
        m1=4; lc1=0.5; I1=0.3; mx1=8; my1=0.3; mth1=0.7;
        m2=4; lc2=0.5; I2=0.3; mx2=8; my2=0.3; mth2=0.7;
        
        xG;
        uG;
    end
    
    methods
        function obj = UnderwaterAcrobotPlant
            obj = obj@Manipulator(2,1);
            obj = setInputLimits(obj,-50,50);
            
            obj = setInputFrame(obj,CoordinateFrame('UnderwaterAcrobotInput',1,'u',{'tau'}));
            obj = setStateFrame(obj,CoordinateFrame('UnderwaterAcrobotState',4,'x',{'theta1','theta2','theta1dot','theta2dot'}));
            obj = setOutputFrame(obj,obj.getStateFrame);
            
            
            obj = setParamFrame(obj,CoordinateFrame('UnderwaterAcrobotParams',9,'p',...
                { 'g','rho','L1','L2','d1','d2','mr1','mr2','cd'}));
            obj = setParamLimits(obj,zeros(obj.getParamFrame.dim,1));
            
            % Disable deriving these parameters to instead match the URDF file
            %Link 1 parameters 
            %[obj.m1,obj.lc1,obj.I1,obj.mx1,obj.my1,obj.mth1] = deriveLinkParams(obj,obj.L1,obj.d1,obj.mr1,obj.rho);
            %Link 2 parameters - using simple URDF values instead of deriving
            %[obj.m2,obj.lc2,obj.I2,obj.mx2,obj.my2,obj.mth2] = deriveLinkParams(obj,obj.L2,obj.d2,obj.mr2,obj.rho);
            
            %Goal position should be unstable eq point
            [~,xUnstable] = eqPoints(obj);
            
            obj.xG = Point(obj.getStateFrame,xUnstable);
            obj.uG = Point(obj.getInputFrame,0);
            
        end
        function [m,lc,I,mx,my,mth] = deriveLinkParams(~,L,d,mr,rho)
            %Sets derived parameters, helper function for constructor
            r = d/2;
            
            %Mass parameters          
            m = mr*rho*pi*r^2*L;
            lc = L/2;
            I = 1/12*m*L^2;
            
            %Added mass coefficients
            mx = rho*pi*r^2*L;
            my = 2/3*rho*pi*r^3;
            mth = 1/12*rho*pi*r^2*L^3;
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
            d1 = obj.d1; mr1=obj.mr1;
            %Unpack link 2
            m2=obj.m2; mx2=obj.mx2; my2 = obj.my2; I2 = obj.I2; mth2 = obj.mth2; lc2=obj.lc2; L2 = obj.L2;
            d2 = obj.d2; mr2=obj.mr2;
            
            g=obj.g; cd = obj.cd; rho = obj.rho;
            
            th1 = q(1,:); th2 = q(2,:);
            w1 = qd(1,:); w2 = qd(2,:);
            s1 = sin(th1); s2 = sin(th2); c2 = cos(th2);
            s12 = sin(th1+th2);
            
            %Drag terms (simplier to see)
            v1x = w1*lc1;
            v2x = L1*w1.*c2+lc2*(w1+w2);
            v2y = -L1*w1.*s2;
            Dx1 = 1/2*rho*L1*d1*v1x.*abs(v1x)*cd; %Drag force in link1 x body frame
            Dx2 = 1/2*rho*L2*d2*v2x.*abs(v2x)*cd; %Drag force in link2 x body frame
            Dy2 = 1/2*rho*pi*d2^2/4*v2y.*abs(v2y)*cd; %Drag force in link2 y body frame
            
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
            C = [C1; C2]+[G1; G2]+[D1;D2]+[obj.b1;obj.b2].*qd;
            
            B = [0; 1];
            
            
        end
        
        function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
            f = dynamics@Manipulator(obj,t,x,u);
            if (nargout>1)
                df = dynamicsGradients(obj,t,x,u,1);
                %[~,df] = geval('dynamics',obj,t,x,u,struct('grad_method','taylorvar'));
                if (nargout>2)
                    % Analytic gradients did not solve for n>2, so using geval
                    [~,~,d2f,d3f] = geval('dynamics',obj,t,x,u,struct('grad_method','taylorvar'));
                end
            end
        end
        
        function [c,V] =balanceLQR(obj)
            Q = diag([1,1,1,1]); R = 1;
            if (nargout<2)
                c = tilqr(obj,obj.xG,obj.uG,Q,R);
            else
                if any(~isinf([obj.umin;obj.umax]))
                    error('currently, you must disable input limits to estimate the ROA');
                end
                [c,V] = tilqr(obj,obj.xG,obj.uG,Q,R);
                pp = feedback(obj.taylorApprox(0,obj.xG,obj.uG,3),c);
                options.method='binary';
                V=regionOfAttraction(pp,V,options);
            end
        end
        
        function [utraj,xtraj]=swingUpTrajectory(obj)
            [x0,~]=eqPoints(obj); tf0 = 4; xf = double(obj.xG);
            
            N = 21;
            prog = DircolTrajectoryOptimization(obj,N,[2 6]);
            prog = prog.addStateConstraint(ConstantConstraint(x0),1);
            prog = prog.addStateConstraint(ConstantConstraint(xf),N);
            prog = prog.addRunningCost(@cost);
            prog = prog.addFinalCost(@finalCost);
            
            traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
            
            for attempts=1:10
              tic
              [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
              toc
              if info==1, break; end
            end

            
            function [g,dg] = cost(dt,x,u)
                R = 1;
                g = sum((R*u).*u,1);
                dg = [zeros(1,1+size(x,1)),2*u'*R];
            end
            
            function [h,dh] = finalCost(t,x)
                h = t;
                dh = [1,zeros(1,size(x,1))];
            end
            
        end
    end
    
end

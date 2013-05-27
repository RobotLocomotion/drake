classdef LinearInvertedPendulum < LinearSystem
  % This is the three-dimensional model of the ZMP dynamics of the linear
  % inverted pendulum as described in Kajita03.
  %
  % - Here the input is the commanded linear *accelerations* of 
  % the center of mass (note that Kajita03 uses the jerk, but I don't think
  % it's warranted).  
  % - The state is [x,y,xdot,ydot]^T.  
  % - The output is the state and the [x,y] position of the ZMP.  
  
  methods 
    function obj = LinearInvertedPendulum(h,g)
      % @param h the (constant) height of the center of mass
      % @param g the scalar gravity @default 9.81
      if (nargin<2) g=9.81; end
      if(isa(h,'numeric'))
        rangecheck(h,0,inf);
        Czmp = [eye(2),zeros(2)];
        C = [eye(4);Czmp];
        Dzmp = -h/g*eye(2);
        D = [zeros(4,2);Dzmp];
      elseif(isa(h,'PPTrajectory'))
        Czmp = [eye(2),zeros(2)];
        C = [eye(4);Czmp]
        h_breaks = h.getBreaks;
        ddh = fnder(h,2);
        Dzmp_val = -h.eval(h_breaks)./(g+ddh.eval(h_breaks));
        D_val = zeros(6,2,length(h_breaks));
        D_val(5,1,:) = Dzmp_val; D_val(6,2,:) = Dzmp_val;
        D = PPTrajectory(spline(h_breaks,D_val));
      else
        error('LinearInvertedPendulum: unknown type for argument h.')
      end
             
      obj = obj@LinearSystem([zeros(2),eye(2);zeros(2,4)],[zeros(2);eye(2)],[],[],C,D);
      obj.h = h;
      obj.g = g;
      
      cartstateframe = CartTableState;
      obj = setInputFrame(obj,CartTableInput);
      sframe = SingletonCoordinateFrame('LIMPState',4,'x',{'x_com','y_com','xdot_com','ydot_com'});
      if isempty(findTransform(sframe,cartstateframe))
        addTransform(sframe,AffineTransform(sframe,cartstateframe,sparse([7 8 15 16],[1 2 3 4],[1 1 1 1],16,4),zeros(16,1)));
        addTransform(cartstateframe,AffineTransform(cartstateframe,sframe,sparse([1 2 3 4],[7 8 15 16],[1 1 1 1],4,16),zeros(4,1)));
      end
      obj = setStateFrame(obj,sframe);
      
      zmpframe = CoordinateFrame('ZMP',2,'z',{'x_zmp','y_zmp'});
      obj = setOutputFrame(obj,MultiCoordinateFrame({sframe,zmpframe}));
    end
    
    function v = constructVisualizer(obj)
      v = CartTableVisualizer;
    end
    
    function varargout = lqr(obj,com0)
      % objective min_u \int dt [ x_zmp(t)^2 ] 
      varargout = cell(1,nargout);
      options.Qy = diag([0 0 0 0 1 1]);
      if (nargin<2) com0 = [0;0]; end
      [varargout{:}] = tilqr(obj,Point(obj.getStateFrame,[com0;0*com0]),Point(obj.getInputFrame),zeros(4),zeros(2),options);
    end
    
    function [c,Vt] = ZMPtracker(obj,dZMP,options)
      if nargin<3 options = struct(); end
      if ~isfield(options,'use_mex') options.use_mex = true; end
      if options.use_mex
        if isfield(options,'dCOM') || ~isTI(obj) || ~isa(dZMP,'PPTrajectory')
          warning('mex not implemented for these options yet');
          options.use_mex = false;
        end
      end        
      
      typecheck(dZMP,'Trajectory');
      dZMP = dZMP.inFrame(desiredZMP);
      
      zmp_tf = dZMP.eval(dZMP.tspan(end));
      if(isTI(obj))
        [~,V] = lqr(obj,zmp_tf);
      else
        % note: i've merged this in from hongkai (it's a time crunch!), but i don't agree with
        % it. it's bad form to assume that the tv system is somehow stationary after D.tspan(end).  
        % - Russ
        valuecheck(dZMP.tspan(end),obj.D.tspan(end)); % assume this
        t_end = obj.D.tspan(end);
        ti_obj = LinearSystem(obj.Ac.eval(t_end),obj.Bc.eval(t_end),[],[],obj.C.eval(t_end),obj.D.eval(t_end));
        ti_obj = setStateFrame(ti_obj,obj.getStateFrame());
        ti_obj = setInputFrame(ti_obj,obj.getInputFrame());
        ti_obj = setOutputFrame(ti_obj,obj.getOutputFrame());
        Q = zeros(4);
  		  options.Qy = diag([0 0 0 0 1 1]);
        [~,V] = tilqr(ti_obj,Point(ti_obj.getStateFrame,[zmp_tf;0*zmp_tf]),Point(ti_obj.getInputFrame),Q,zeros(2),options);
      end
      
      options.tspan = linspace(dZMP.tspan(1),dZMP.tspan(2),10);
      options.sqrtmethod = false;
      if(isfield(options,'dCOM'))
%        options.dCOM = setOutputFrame(options.dCOM,obj.getStateFrame);
        options.dCOM = options.dCOM.inFrame(obj.getStateFrame);
        options.xdtraj = options.dCOM;
        Q = eye(4);
      else
        Q = zeros(4);
      end

      if options.use_mex 
        % ok, not actually mex yet... but should be optimized considerably
        ts = options.tspan;
        
        % cost ((zmp-zmp_des)'*(zmp-zmp_des) + u'*R*u)
        %   with zmp = x(1:2) -h/g*u
        %   and everything in zmp - zmp_tf coordinates (necessary for
        %   translational invariance.. otherwise the numerics get dicey).
        %  =>  cost x'*Q*x + x'*q2 + u'*R*u + u'*r2 + 2*x'*N*u + r3 with
        %   Q = diag([1 1 0 0]);  q2 = -2*[zmp_des;0;0]
        %   R = R + (h/g)^2*eye(2);  r2 = 2*zmp_des*h/g;
        %   N = -h/g*[eye(2);zeros(2)];  r3 = zmp_des'*zmp_des
        hg = obj.h/obj.g;
        A = obj.Ac; B = obj.Bc; 
        Q = diag([1 1 0 0]);
        R = hg^2*eye(2); Ri = inv(R);
        N = -hg*[eye(2);zeros(2)];
        
        % commented out to leave V in relative coordinates from the lqr
        % solution
%        V = V.inFrame(getStateFrame(obj));
        
        [ts,S] = ode45(@(t,x)zmpRiccati(t,x,zmp_tf),dZMP.tspan([end,1]),[V.S(:);V.s1;V.s2]);
        ts = flipud(ts); S = flipud(S)';
        m = length(ts);

        % flip S back to world coordinates
        % vectorized version of this:
        %  c = [-zmp_tf;0;0];
        %  S(17:21,i) = [ (S1+S1')*c + S2; c'*S1*c + c'*S2 + S3 ];
%        S(17,:) = S(17,:) - 2*S(1,:)*zmp_tf(1) - (S(2,:)+S(5,:))*zmp_tf(2);
%        S(18,:) = S(18,:) - (S(2,:)+S(5,:))*zmp_tf(1) - 2*S(6,:)*zmp_tf(2);
%        S(21,:) = S(21,:) + zmp_tf'*[S(1,:)*zmp_tf(1) + S(5,:)*zmp_tf(2);S(2,:)*zmp_tf(1) + S(6,:)*zmp_tf(2)];
        
        Sdot=S; % preallocate
        for i=1:length(ts)
          S1 = reshape(S(1:16,i),[4 4]);  % flip S back to world coordinates
          S2 = S(17:20,i);
          S3 = S(21,i);
          c = [-zmp_tf;0;0];
          S(17:21,i) = [ (S1+S1')*c + S2; c'*S1*c + c'*S2 + S3 ];
          Sdot(:,i) = zmpRiccati(ts(i),S(:,i),zeros(2,1));
%          Sdot1 = reshape(Sdot(1:16,i),[4 4]);
%          Sdot2 = Sdot(17:20,i);
%          Sdot3 = S(21,i);
%          Sdot(17:21,i) = [ (Sdot1+Sdot1')*c + Sdot2; c'*Sdot1*c + c'*Sdot2 + Sdot3 ];
        end
        
        K1 = reshape(-Ri*(B'*reshape(S(1:16,:),[4 4*m]) + repmat(N',[1 m])),[2 4 m]);
        r2 = 2*eval(dZMP,ts)*hg;
        K2 = reshape(-.5*Ri*(B'*reshape(S(17:20,:),[4 m]) + r2),[2 m]);
        Kdot1 = reshape(-Ri*(B'*reshape(Sdot(1:16,:),[4 4*m]) + repmat(N',[1 m])),[2 4 m]);
        r2dot = 2*deriv(dZMP,ts)*hg;
        Kdot2 = reshape(-.5*Ri*(B'*reshape(Sdot(17:20,:),[4 m]) + r2dot),[2 m]);
        Kpp1 = pchipDeriv(ts,K1,Kdot1,Kdot1,[2 4]);
        Kpp2 = pchipDeriv(ts,K2,Kdot2,Kdot2,[2 1]);

        c = AffineSystem([],[],[],[],[],[],[],PPTrajectory(Kpp1),PPTrajectory(Kpp2)); 
        c = setInputFrame(c,getStateFrame(obj));
        c = setOutputFrame(c,getInputFrame(obj));
        
        if (nargout>1)
          Spp1 = pchipDeriv(ts,S(1:16,:),Sdot(1:16,:),[],[4 4]);
          Spp2 = pchipDeriv(ts,S(17:20,:),Sdot(17:20,:),[],[4 1]);
          Spp3 = pchipDeriv(ts,S(21,:),Sdot(21,:));
          Vt = QuadraticLyapunovFunction(getStateFrame(obj),PPTrajectory(Spp1),PPTrajectory(Spp2),PPTrajectory(Spp3));
        end
      else
        dZMP = setOutputFrame(dZMP,getFrameByNum(getOutputFrame(obj),2)); % override it before sending in to tvlqr
        % note: the system is actually linear, so x0traj and u0traj should
        % have no effect on the numerical results (they just set up the
        % frames)
        x0traj = setOutputFrame(ConstantTrajectory([zmp_tf;0;0]),obj.getStateFrame);
        u0traj = setOutputFrame(ConstantTrajectory([0;0]),obj.getInputFrame);
        options.Qy = diag([0 0 0 0 1 1]);
        options.ydtraj = [x0traj;dZMP];
        WS = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
        [c,Vt] = tvlqr(obj,x0traj,u0traj,Q,zeros(2),V,options);
        warning(WS);
      end
      
        function Sdotvec = zmpRiccati(t,Svec,zmp0)
          S1 = reshape(Svec(1:16),[4 4]);
          S2 = reshape(Svec(17:20),[4 1]);
          S3 = Svec(21);
          zmp_des = eval(dZMP,t) - zmp0;
          q2 = [-2*zmp_des;0;0];
          r2 = 2*zmp_des*hg;
          q3 = zmp_des'*zmp_des;
          
          Sdot1 = -(Q - (N+S1*B)*Ri*(B'*S1+N') + S1*A + A'*S1);
          rs = (r2+B'*S2)/2;
          Sdot2 = -(q2 - 2*(N+S1*B)*Ri*rs + A'*S2);
          Sdot3 = -(q3 - rs'*Ri*rs);
          
          Sdotvec = [Sdot1(:); Sdot2; Sdot3];
          
          if (0)
          c = [zmp_tf-zmp0;0;0];
          Qt{1} = Q;
          Qt{2} = (Q+Q')*c + q2;
          Qt{3} = c'*Q*c + c'*q2 +q3;
          Rt{1} = R;
          Rt{2} = (r2' + 2*c'*N)';
          Rt{3} = 0;
          St{1} = S1; St{2} = (S1+S1')*c + S2; St{3} = c'*S1*c + c'*S2 + S3;
          Stdot{1} = Sdot1;  Stdot{2}=(Sdot1+Sdot1')*c + Sdot2; Stdot{3}=c'*Sdot1*c + c'*Sdot2 + Sdot3;
%          plot(t,St{2}(1),'b.');
          if (0) %9.95<=t & t<10)
            t
            disp(['Qt{1} = ',mat2str(Qt{1})]);
            disp(['Qt{2} = ',mat2str(Qt{2})]);
            disp(mat2str(-2*zmp_des + 2*zmp_tf));
            disp(['Qt{3}+Rt{3} = ',mat2str(Qt{3}+Rt{3})]);
            disp(['Rt{1} = ',mat2str(Rt{1})]);
            disp(['Rt{2} = ',mat2str(Rt{2})]);
%            N
%            A
%            B
            disp(['St{1} = ',mat2str(St{1})]);
            disp(['St{2} = ',mat2str(St{2})]);
            disp(['St{3} = ',mat2str(St{3})]);
            disp(['Stdot{1} = ',mat2str(Stdot{1})]);
            disp(['Stdot{2} = ',mat2str(Stdot{2})]);
            disp(['Stdot{3} = ',mat2str(Stdot{3})]);
          end
          end
          
        end
      
    end
    
    function comtraj = COMplanFromTracker(obj,com0,comdot0,tspan,c)
      doubleIntegrator = LinearSystem([zeros(2),eye(2);zeros(2,4)],[zeros(2);eye(2)],[],[],eye(4),zeros(4,2));
      doubleIntegrator = setInputFrame(doubleIntegrator,getInputFrame(obj));
      doubleIntegrator = setStateFrame(doubleIntegrator,getStateFrame(obj));
      doubleIntegrator = setOutputFrame(doubleIntegrator,getStateFrame(obj));  % looks like a typo, but this is intentional: output is only the LIMP state
      sys = feedback(doubleIntegrator,c);
      
      comtraj = simulate(sys,tspan,[com0;comdot0]);
      comtraj = inOutputFrame(comtraj,sys.getOutputFrame);
      comtraj = comtraj(1:2);  % only return position (not velocity)
    end
    
    function comtraj = COMplan(obj,com0,comf,dZMP,options)
      % implements analytic method from Harada06
      %  notice the different arugments (comf instead of comdot0)
      sizecheck(com0,2);
      sizecheck(comf,2);
      typecheck(dZMP,'PPTrajectory');  
      dZMP = dZMP.inFrame(desiredZMP);
      if ~isnumeric(obj.h) error('variable height not implemented yet'); end
      
      valuecheck(obj.g,9.81);
 
      comtraj = LinearInvertedPendulum.COMtrajFromZMP(obj.h,com0,comf,dZMP.pp);
%      com_pp = LinearInvertedPendulum.COMsplineFromZMP(obj.h,com0,comf,dZMP.pp);
%      comtraj = PPTrajectory(com_pp);
    end
    
    function comtraj = ZMPplanner(obj,com0,comdot0,dZMP,options)
      warning('Drake:DeprecatedMethod','The ZMPPlanner function is deprecated.  Please use COMplan (using the closed-form solution) or ZMPPlanFromTracker'); 
    
      % got a com plan from the ZMP tracking controller
      if(nargin<5) options = struct(); end
      c = COMtracker(obj,dZMP,options);
      comtraj = COMplanFromTracker(obj,com0,comdot0,dZMP.tspan,c);
    end
    
    
  end
  
  methods (Static)
    function comtraj = COMtrajFromZMP(h,com0,comf,zmp_pp)
      [breaks,coefs,l,k,d] = unmkpp(zmp_pp);
      for i=1:2
        this_zmp_pp = mkpp(breaks,coefs(i:2:end,:));
        ct{i} = LinearInvertedPendulum2D.COMtrajFromZMP(h,com0(i),comf(i),this_zmp_pp);
      end
      comtraj = vertcat(ct{1},ct{2});
    end
    
    function com_pp = COMsplineFromZMP(h,com0,comf,zmp_pp)
      % fast method for computing closed-form ZMP solution (from Harada06)
      % note: there is no error checking on this method (intended to be fast).
      % use ZMPplan to be more safe.
      
      [breaks,coefs,l,k,d] = unmkpp(zmp_pp);
      
      for i=1:2
        this_zmp_pp = mkpp(breaks,coefs(i:2:end,:));
        [V,W,A,Tc,~,comdot0,comdotf] = LinearInvertedPendulum2D.AnalyticZMP_internal(h,com0(i),comf(i),this_zmp_pp);
      
        % equation (5)
        % Note: i could return a function handle trajectory which is exact
        % (with the cosh,sinh terms), but for now I think it's sufficient and more
        % practical to make a new spline.
        com_knots(i,:) = [comdot0,V+A(1,:),comf(i),comdotf];
      end
      com_pp = csape(breaks,com_knots,[1 1]);
      
      % NOTEST
    end
    
    function horz_comddot_d = COMaccelerationFromZMP(h,com0,comf,dZMP)
      % fast method for computing instantaneous COM acceleration (using
      % Harada06)

      [breaks,coefs,l,k,d] = unmkpp(zmp_pp);
      
      for i=1:2
        this_zmp_pp = mkpp(breaks,coefs(i,:,:),1);
        [V,W,A,Tc] = LinearInvertedPendulum2D.AnalyticZMP_internal(h,com0(i),comf(i),this_zmp_pp);
        
        horz_comddot_d(i) = V(1)*Tc^2 + 2*A(3,1);
      end
      
      % NOTEST
    end
    
    function runPassive
      r = LinearInvertedPendulum(1.055);
      v = r.constructVisualizer();
      ytraj = r.simulate([0 3],[0;0;.1;0]);
      v.playback(ytraj);
    end
    
    function runLQR
      r = LinearInvertedPendulum(1.055);
      c = lqr(r);
      output_select(1).system = 1;
      output_select(1).output = 1;
      sys = mimoFeedback(r,c,[],[],[],output_select);
      
      v = r.constructVisualizer();

      for i=1:5;
        ytraj = sys.simulate([0 5],randn(4,1));
        v.playback(ytraj);
      end
    end
    
    function ZMPtrackingDemo()
      r = LinearInvertedPendulum(1.055);
      ts = linspace(0,8,100); 
      dZMP = setOutputFrame(PPTrajectory(spline(ts,[.08*sin(1.5*ts*(2*pi));.25*sin(1.5*ts*(2*pi))])),desiredZMP);
      c = ZMPtracker(r,dZMP);
      output_select(1).system = 1;
      output_select(1).output = 1;
      sys = mimoFeedback(r,c,[],[],[],output_select);

      v = r.constructVisualizer();
      
      ytraj = sys.simulate(dZMP.tspan,randn(4,1));
      v.playback(ytraj);
    end
  end
  
  properties
    h
    g
  end
  
end

classdef SoftPaddleControl < DrakeSystem
  properties
    kp
    kd
    plant
    numerically_stable
    K
  end
  
  methods
    function obj = SoftPaddleControl(plant)
      %obj = obj@HybridDrakeSystem(2*plant.in_contact.num_positions+1,size(plant.in_contact.B,2));
      obj = obj@DrakeSystem(0,0,9,1,true,true);
      obj = setInputFrame(obj, getOutputFrame(plant));
      obj = setOutputFrame(obj, getInputFrame(plant));
      obj.kp = 1000;
      obj.kd = 2*sqrt(obj.kp);
      obj.plant = plant;
      
      %Q = diag([10,1,1,10]); R = 1; Qf = diag([10,1,0,10]);
      
      A =  [-0.499901913522657,-0.0142120087675696,0.645299810000860,1.80411241501588e-12;0.000819073099123102,0.00880311712148085,-0.000867456499520358,-8.88178419700125e-13;-1.53727627522300,-0.00898219706924053,-0.0158032333929491,1.60635393875452e-12;-0.0248725908136294,-3.06321655453878,-0.0293840063276818,-2.39808173319034e-11];
      B=   [11.4912288748530;-2.94788491242493;11.7902392734902;-9.20713654905914];
      Q = diag([1,1,1,1]);
      R = 1e-3;
      %R2 = 1e-3*diag([1,1]);
      [K,S,E] = dlqr(A,B,Q,R);
      %K = [-0.071799     0.042213     0.022939   4.7217e-13];
      obj.K = -K; %TODO: Fix the code to not need the sign inversion
      %[K2,S2,E2] = dlqr(A,B2,Q,R2);
      
      % xtraj and utraj to be defined
      %             c = tvlqr(plant,xtraj,utraj,Q,R,Qf);
      obj.numerically_stable = false;
    end
    
    function u = output(obj,t,~,x)
      m = x(1);
      q = x(2:5);
      qp = x(6:9);
      
      [H,C,B] = manipulatorDynamics(obj.plant.no_contact, q, qp);
      [phi,J,ddphi] = obj.plant.in_contact.position_constraints{1}.eval(q);
      
      Jp = reshape(ddphi,length(q),[])*qp;
      Hinv = inv(H);
      Hinvtilde = J*Hinv*J';
      %             Hinvtilde = J*(H\J');
      
      Delta = Hinvtilde - J(1)*J*Hinv*B;
      %             Delta = Hinvtilde - J(1)*J*(H\B);
      
      [T,U] = energy(obj.plant,x);
      Eref = 121;
      Etilde = T+U-Eref;
      uE = -10*qp(1)*Etilde;
      
      epsilon = 0.5;
      xFixed = -0.031;
      xFixed = -0.05;
      zTouchDes = 4;
      %             psid = -0.001*(q(3)-xFixed)-0.005*qp(3);
      
      psi = q(1);
      R = [cos(psi), sin(psi); -sin(psi), cos(psi)];
      load_in_paddle = -[3; 0] + R'*([q(3); q(4)]-[-3;3]);
      loadv_in_paddle = R'*[qp(3); qp(4)];
      
      tf = max(roots([-1/2*9.81*cos(psi), loadv_in_paddle(2), load_in_paddle(2)-1]));
      zTouch = q(4) + qp(4)*tf - 1/2*9.81*tf^2;
      zpTouch = qp(4) - 9.81*tf;
      xTouch = q(3) + qp(3)*tf;
      xpTouch = qp(3);
      
      %             K = [0.066513, 0.071367, 0.032835, 1.3774e-11];   % original
      %             K = 0.5*[0.11276, -0.095662, -0.047351, 2.8845e-11];
      %             K = 0.75*[0.11155, -0.09665, -0.048202, 5.6288e-12];  % Works nicely
      %K = -[-0.071799     0.042213     0.022939   4.7217e-13];
      
      psid = obj.K(1)*(xTouch-xFixed) + obj.K(2)*(zTouch-zTouchDes) + obj.K(3)*xpTouch + obj.K(4)*(zpTouch);
      %             psid = -psid;
      %             psid = 0;
      
      u = -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1);
      if m == 2
        if(obj.numerically_stable)
          u = Hinvtilde/(Delta)*u + J(1)*((Jp'+2/epsilon*J)*qp+1/(epsilon^2)*phi-J*Hinv*C)/(Delta);
        else
          %                     psid = -0.0005*(q(3)-xFixed)-0.001*qp(3);
          %                     if psid > 0.01
          %                         psid = 0.01;
          %                     elseif psid < -0.01
          %                         psid = -0.01;
          %                     end
          psid = evalin('base','psid');
          u = -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1);
          u = Hinvtilde/(Delta)*u + J(1)*(Jp'*qp-J*Hinv*C)/(Delta);
          %                     if q(1)*qp(1) < 0
          %                         u = u + uE;
          %                     end
        end
      end
      assignin('base','psid', psid)
      
      tCutOff1 = 60;
      tCutOff2 = 100;
      if t > tCutOff1+50 && t < tCutOff2
        %             psid = 0;
        %             psid = -0.0005*q(3)-0.001*qp(3);
        %K = 0.75*[0.11155, -0.09665, -0.048202, 5.6288e-12];
        psid = obj.K(1)*(xTouch-xFixed) + obj.K(2)*(zTouch-zTouchDes) + obj.K(3)*xpTouch + obj.K(4)*(zpTouch);
        u = -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1) - 0.1*sign(Etilde*qp(1));
        %             u = 1000*Etilde*qp(1);
      end
      
      if t > tCutOff1 && t < tCutOff2 && m == 2
        psi = q(1);
        R = [cos(psi), sin(psi); -sin(psi), cos(psi)];
        load_in_paddle = -[3; 0] + R'*([q(3); q(4)]-[-3;3]);
        z = load_in_paddle(2);
        %                 fprintf(['z = ', num2str(z), '\n'])
        fprintf(['t = ', num2str(t), '\n'])
        
        k1 = 0.1;
        k2 = 1e+3;
        lambdad = 1/Hinvtilde*(  k1*J*Hinv*B*sign(Etilde*qp(1)) + J*Hinv*C + Jp'*qp  );
        
        epsilon = 1e-2;
        cf = 1/2 + 1/pi*atan(1/epsilon*(1-z));
        gammad = cf*lambdad;
        
        lambda = max(0,gammad);
        %                 u = -1/(J*Hinv*B)*(Hinvtilde*lambda - J*Hinv*C - Jp'*qp);
        %                 u = -cf*k*sign(Etilde*qp(1));
        %                 u = -cf*k*2/pi*atan(20*Etilde*qp(1)) + C(1);
        %                 u = -cf*k*Etilde*qp(1) + C(1);
        %                 psid = evalin('base','psid');
        %                 if abs(Etilde) > 1
        %                     u = Hinvtilde/(Delta)*( -obj.kp*(q(1)-psid) - obj.kd*qp(1) ) + J(1)*(Jp'*qp-J*Hinv*C)/(Delta) - k1*2/pi*atan(20*Etilde*qp(1));
        u = Hinvtilde/(Delta)*( -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1) ) + J(1)*(Jp'*qp-J*Hinv*C)/(Delta) - k2*Etilde*qp(1);
        %                 else
        %                     u = Hinvtilde/(Delta)*( -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1) ) + J(1)*(Jp'*qp-J*Hinv*C)/(Delta);
        %                 end
        %                 if t > 40.02
        %                     keyboard
        %                 end
      end
      assignin('base','psid', psid)
      
      %% Set
      %             u = C(1);
      
    end
    
    %         function z = mySign(y, M)
    %             z = 2/pi*atan(M*y);
    %         end
  end
  
  methods (Static)
    function run()
      p = SoftPaddleHybrid();
      porig = p;
      plantSim = SimulinkModel(p.getModel());
      % take the frames from the simulink model and use those for the simulation of plant and controller
      p = setOutputFrame(p, getOutputFrame(plantSim));
      p = setInputFrame(p, getInputFrame(plantSim));
      
      c = SoftPaddleControl(p);
      
      output_select(1).system = 1;
      output_select(1).output = plantSim.getOutputFrame();
      output_select(2).system = 2;
      output_select(2).output = c.getOutputFrame();
      
      sys = mimoFeedback(plantSim,c,[],[],[],output_select);
      
      x0 = p.getInitialState();
      
      tic
      [ytraj,xtraj] = simulate(sys,[0 100],x0);
      toc
      
      utraj = ytraj(10);
      utraj = utraj.setOutputFrame(getInputFrame(porig));
      xtraj = xtraj.setOutputFrame(getOutputFrame(porig));
      v = porig.constructVisualizer();
      v.drawWrapper(0,x0);
      
      v.playback(xtraj,struct('slider',true));
      
      tt=getBreaks(xtraj);
      
      save('trajectorytoStabilizeShort.mat','utraj','xtraj','tt');
      
      if (1)
        % energy / cable length plotting
        % note, set alpha=0 in Manipulator/computeConstraintForce to reveal
        % some artifacts, especially at the release guard when
        % the pulley effectively hits a hard-stop.  this is due to the fact
        % that phidot>0 during the in_contact phase.  it's hard to tell if
        % this is numerical artifact or a bad gradient in CableLength.
        
        E=tt;
        cl=tt;
        nq=getNumPositions(porig.no_contact);
        dcl=zeros(length(tt),1,nq);
        ddcl=zeros(length(tt),1,nq*nq);
        for i=1:length(tt)
          x = xtraj.eval(tt(i));
          
          %           if i 50>= length(tt)/20 keyboard; end
          [T,U] = energy(porig,x);
          E(i)= T+U;
          if (x(1)==1) %flight mode
            [cl(i),dcl(i,1,:),ddcl(i,1,:)]=porig.no_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
          else
            [cl(i),dcl(i,1,:),ddcl(i,1,:)]=porig.in_contact.position_constraints{1}.fcn.eval(x((1:nq)+1));
          end
        end
        y = ytraj.eval(tt);
        figure(1); clf;
        subplot(3,1,1); plot(tt,E, 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\mathcal{H}$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,2); plot(tt,cl, 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\phi(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,3); plot(tt,y(10,:), 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$u$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        figure(3); clf;
        subplot(3,1,1); plot(tt,cl, 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\phi(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,2); plot(tt,dcl(:,1,4), 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\dot{\phi}(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(3,1,3); plot(tt,ddcl(:,1,16), 'LineWidth', 2);
        axis tight
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\ddot{\phi}(q)$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        t = linspace(xtraj.tspan(1), xtraj.tspan(end), 1001);
        x = eval(xtraj, t);
        figure(2), clf
        subplot(2,2,1)
        plot(t,x(3,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\theta$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,2)
        plot(t,x(4,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$x$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,3)
        plot(t, x(5,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$z$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        subplot(2,2,4)
        plot(t, x(2,:), 'LineWidth', 2)
        axis('tight')
        xlabel('$t$ [sec]', 'Interpreter', 'LaTeX', 'FontSize', 15)
        ylabel('$\psi$', 'Interpreter', 'LaTeX', 'FontSize', 15)
        
        %                 poincareLinearMirrorLawControl
        
      end
    end
    
  end
end
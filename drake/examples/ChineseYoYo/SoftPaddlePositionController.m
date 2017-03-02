classdef SoftPaddlePositionController < DrakeSystem
    properties
        kp
        kd
        plant
        psiDes
    end
    
    methods
        function obj = SoftPaddlePositionController(plant)
            %obj = obj@HybridDrakeSystem(2*plant.in_contact.num_positions+1,size(plant.in_contact.B,2));
            obj = obj@DrakeSystem(0,0,9,1,true,true);
            obj = setInputFrame(obj, getOutputFrame(plant));
            obj = setOutputFrame(obj, getInputFrame(plant));
            obj.kp = 1000;
            obj.kd = 2*sqrt(obj.kp);
            obj.plant = plant;
            obj.psiDes = 0;
        end
       
        function u = output(obj,~,~,x)
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

            psid = obj.psiDes;
            u = -obj.kp*(q(1)-psid) - obj.kd*qp(1) + C(1);
            if m == 2
              u = Hinvtilde/(Delta)*u + J(1)*(Jp'*qp-J*Hinv*C)/(Delta);
            end
            
        end
        
    end
    
    methods (Static)
        function run()
            p = SoftPaddleHybrid();
            porig = p;
            plantSim = SimulinkModel(p.getModel());
            % take the frames from the simulink model and use those for the simulation of plant and controller 
            p = setOutputFrame(p, getOutputFrame(plantSim));
            p = setInputFrame(p, getInputFrame(plantSim));
            
            c = SoftPaddlePositionController(p);
                      
            output_select(1).system = 1;
            output_select(1).output = plantSim.getOutputFrame();
            output_select(2).system = 2;
            output_select(2).output = c.getOutputFrame();
            
            sys = mimoFeedback(plantSim,c,[],[],[],output_select);
            x0 = Point(getStateFrame(p));
            x0.m = 1;
            x0.load_x = -0.02905;
            x0.load_z = 3.9999;
            x0 = double(x0);
            x0(2:end) = resolveConstraints(p.in_contact,x0(2:end));
            
            tic
            [ytraj,xtraj] = simulate(sys,[0 1.04],x0);
            toc
            
            utraj = ytraj(10);
            utraj = utraj.setOutputFrame(getInputFrame(porig));
            xtraj = xtraj.setOutputFrame(getOutputFrame(porig));
            v = porig.constructVisualizer();
            v.drawWrapper(0,x0);
            
            v.playback(xtraj,struct('slider',true));

            tt=getBreaks(xtraj);

%             save('trajectorytoStabilizeShort.mat','utraj','xtraj','tt');
            
            if (0)
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
                
            end
        end
        
    end
end
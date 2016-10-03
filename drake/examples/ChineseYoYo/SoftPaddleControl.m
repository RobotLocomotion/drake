classdef SoftPaddleControl < DrakeSystem
    properties
        kp
        kd
        plant
    end
    
    methods
        function obj = SoftPaddleControl(plant)
            %obj = obj@HybridDrakeSystem(2*plant.in_contact.num_positions+1,size(plant.in_contact.B,2));
            obj = obj@DrakeSystem(0,0,9,1,true,true);
            obj = setInputFrame(obj, getOutputFrame(plant));
            obj = setOutputFrame(obj, getInputFrame(plant));
            obj.kp = 100;
            obj.kd = 2*sqrt(obj.kp);
            obj.plant = plant;
            
            Q = diag([10,1,1,10]); R = 1; Qf = diag([10,1,0,10]);
            % xtraj and utraj to be defined
%             c = tvlqr(plant,xtraj,utraj,Q,R,Qf);
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
            Delta = Hinvtilde - J(1)*J*Hinv*B;
            
            u = -obj.kp*q(1) - obj.kd*qp(1);
            if m == 2
                u = -Hinvtilde/(Delta)*u + J(1)*(Jp'*qp-J*Hinv*C)/(Delta);
            end
        end
    end
    
    methods (Static)
        function run()
            p = SoftPaddleHybrid();
            plantSim = SimulinkModel(p.getModel());
            % take the frames from the simulink model and use those for the simulation of plant and controller 
            p = setOutputFrame(p, getOutputFrame(plantSim));
            p = setInputFrame(p, getInputFrame(plantSim));
            
            c = SoftPaddleControl(p);
                       
            sys = feedback(plantSim,c);
            v = p.constructVisualizer();
            
            x0 = p.getInitialState();
            v.drawWrapper(0,x0);
            [ytraj,xtraj] = simulate(sys,[0 5],x0);
            v.playback(ytraj,struct('slider',true));
        end
    end
end
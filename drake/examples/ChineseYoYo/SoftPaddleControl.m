classdef SoftPaddleControl < HybridDrakeSystem
    properties
        kp
        kd
    end
    
    methods
        function obj = SoftPaddleControl(plant)
            obj = obj@HybridDrakeSystem(2*plant.in_contact.num_positions+1,size(plant.in_contact.B,2));
%             obj = obj@DrakeSystem(0,0,9,1);
            obj = setInputFrame(obj, getOutputFrame(plant));
            obj = setOutputFrame(obj, getInputFrame(plant));
            obj.kp = 100;
            obj.kd = 2*sqrt(obj.kp);
            
            Q = diag([10,1,1,10]); R = 1; Qf = diag([10,1,0,10]);
            % xtraj and utraj to be defined
%             c = tvlqr(plant,xtraj,utraj,Q,R,Qf);
        end
        
        function u = output(obj,t,~,x)
            m = x(1);
            q = x(2:5);
            qp = x(6:9);
            
            [H,C,B] = manipulatorDynamics(obj, q, qp);
            [phi,J,ddphi] = obj.in_contact.position_constraints{1}.eval(q);
            
            Jp = reshape(ddphi,length(q),[])*qp;
            Hinv = inv(H);
            Hinvtilde = J*Hinv*J';
            Delta = Hinvtilde - J(1)*J*Hinv*B;
            
            u = -obj.kp*q(1) - obj.kd*qp(1);
            if m == 2
                u = -Hinvtilde/(Hinvtilde-Delta)*u + J(1)*(Jp*qp-J*Hinv*C)/(Hinvtilde-Delta);
            end
        end
    end
    
    methods (Static)
        function run()
            p = SoftPaddleHybrid();
            c = SoftPaddleControl(p);
            sys = feedback(p,c);
            v = p.constructVisualizer();
            
            x0 = r.getInitialState();
            v.drawWrapper(0,x0);
            [ytraj,xtraj] = simulate(sys,[0 15],x0);
            v.playback(ytraj,struct('slider',true));
        end
    end
end
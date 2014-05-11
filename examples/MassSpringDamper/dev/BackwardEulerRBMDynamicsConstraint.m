classdef BackwardEulerRBMDynamicsConstraint < NonlinearConstraint
  properties(SetAccess = protected)
    manip; % A RigidBodyManipulator object
    n_pos_states;
    n_vel_states;
    n_inputs;
  end
  methods 
      
    function obj = BackwardEulerRBMDynamicsConstraint(r)
      n_pos_states_ = r.getNumDOF();
      n_vel_states_ = r.getNumDOF();
      n_inputs_ = r.getNumInputs();
      obj = obj@NonlinearConstraint(zeros(n_pos_states_+n_vel_states_,1), ...
              zeros(n_pos_states_+n_vel_states_,1),...
              2*(n_pos_states_+n_vel_states_)+n_inputs_+1);
      obj.manip = r;
      obj.n_pos_states = n_pos_states_;
      obj.n_vel_states = n_vel_states_;
      obj.n_inputs = n_inputs_;
    end
    
    function [residual, dresidual] = eval(obj,x)
        states_inputs = x(1:2*(obj.n_pos_states+obj.n_vel_states)+obj.n_inputs);
        h = x(2*(obj.n_pos_states+obj.n_vel_states)+obj.n_inputs+1);
        [residual, dresidual] = evalh(obj,states_inputs,h);
        da = 1e-7;
        dh = (evalh(obj,states_inputs,h+da)-residual)/da;
        dresidual = [dresidual,dh];
    end
    
    function [residual, dresidual] = evalh(obj,x,h)
        
      pos1 = x(1:obj.n_pos_states);
      dpos1 = [eye(obj.n_pos_states), ...
          zeros(obj.n_pos_states, obj.n_pos_states + 2*obj.n_vel_states + obj.n_inputs)];
      vel1 = x(obj.n_pos_states+(1:obj.n_vel_states));
      dvel1 = [zeros(obj.n_vel_states,obj.n_pos_states), ...
               eye(obj.n_vel_states), ...
               zeros(obj.n_vel_states, obj.n_pos_states + obj.n_vel_states + obj.n_inputs)];
      pos2 = x(obj.n_pos_states+obj.n_vel_states+(1:obj.n_pos_states));
      dpos2 = [zeros(obj.n_pos_states,obj.n_pos_states+obj.n_vel_states), ...
               eye(obj.n_pos_states), ...
               zeros(obj.n_pos_states, obj.n_vel_states + obj.n_inputs)];
      vel2 = x(2*obj.n_pos_states+obj.n_vel_states+(1:obj.n_vel_states));
      dvel2 = [zeros(obj.n_vel_states,2*obj.n_pos_states+obj.n_vel_states), ...
               eye(obj.n_vel_states), ...
               zeros(obj.n_vel_states, obj.n_inputs)];
      u2 = x(2*(obj.n_pos_states+obj.n_vel_states)+(1:obj.n_inputs));
      du2 = [zeros(obj.n_inputs,2*(obj.n_pos_states+obj.n_vel_states)), ...
               eye(obj.n_inputs)];
      
      [H2,C2,B2,dH2,dC2,dB2] = manipulatorDynamics(obj.manip,pos2,vel2);
      
%       options.grad_method = 'numerical';
%       [~,~,~,dH2num,dC2num,dB2num] = geval(3,@manipulatorDynamics,obj.manip,pos2,vel2,options);
%       Herr = max(max(abs(dH2num-reshape(dH2,size(H2,1),[]))));
%       Herrp = 100*Herr/max(max(abs([dH2num,reshape(dH2,size(H2,1),[])])));
%       Cerr = max(max(abs(dC2num-reshape(dC2,size(C2,1),[]))));
%       Cerrp = 100*Cerr/ max(max(abs([dC2num,reshape(dC2,size(C2,1),[])])));
%       Berr = max(max(abs(dB2num-reshape(dB2,size(B2,1),[]))));
%       Berrp = 100*Berr/max(max(abs([dB2num,reshape(dB2,size(B2,1),[])])));
%       fprintf('Error in dH: %d = %f percent\n', Herr, Herrp);
%       fprintf('Error in dC: %d = %f percent\n', Cerr, Cerrp);
%       fprintf('Error in dB: %d = %f percent\n', Berr, Berrp);
      
      dH2 = reshape(dH2*[dpos2;dvel2],obj.n_pos_states,obj.n_pos_states*(obj.xdim-1));
      dH2 = blockwiseTranspose(dH2,[obj.n_pos_states,obj.n_pos_states]);
      dC2 = dC2*[dpos2;dvel2];
      dB2 = dB2*[dpos2;dvel2];
      dB2 = reshape(dB2,obj.n_vel_states,obj.n_inputs*(obj.xdim-1));  
      dB2 = blockwiseTranspose(dB2,[obj.n_vel_states,obj.n_inputs]);
     
      residual = [pos2 - (pos1 + h*vel2);
                  H2*(vel2-vel1)/h + C2 - B2*u2];
              
      dresidual = [dpos2 - (dpos1 + h*dvel2);
                   reshape(dH2*(vel2-vel1),obj.n_pos_states,obj.xdim-1)/h + H2*(dvel2-dvel1)/h + dC2 - reshape(dB2*u2,obj.n_pos_states,obj.xdim-1) - B2*du2];
      
    end
  end
end
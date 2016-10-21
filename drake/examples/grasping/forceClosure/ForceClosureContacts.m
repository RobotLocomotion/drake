classdef ForceClosureContacts < ForceClosureContactsBase
  properties(SetAccess = protected)    
    f % Forces. A 3 x obj.num_contacts matrix. f(:,i) is the force at the i'th contact point.
            
    XCCF % XCCF{i} is a 9 x 9 matrix, supposedly to be the bilinear matrix [xc(:,i);c(:,i);f(:,i)]*[xc(:,i);c(:,i);f(:,i)]'
    epsilonG
    
    fc_weights % A obj.num_fc_edges x obj.num_contacts matrix. fc_weights(i,j) is the weight of the i'th edge in the friction cone at j'th contact point.
    fc_QuatWeights % A obj.num_contacts x 1 cell, This is supposed to be the bilinear matrix [tril(fc_Quat{i});fc_weights(:,i)]*[tril(fc_Quat{i};fc_weights(:,i)]'
  end
  
  methods
    function obj = ForceClosureContacts(A_xc,b_xc,num_contacts,mu_face,epsilonG,options)
      % @param A_xc     A 3 X 3 matrix. Refer to obj.A_xc property
      % @param b_xc     A 3 x 1 matrix. Refer to obj.b_xc property
      % @param num_contacts   A scalar. The number of contact points
      % @param mu_face    A scalar. The friction coefficient
      % @param epsilonG   A scalar. The grasp matrix G would be regarded as
      % full rank, if G*G'>=epsilonG*eye(6);
      % @param options    A structure
      %                   -- lin_fc_flag   If true, use linearized friction
      %                   cone, default is false
      %                   -- num_fc_edges  If lin_fc_flag = true, this
      %                   represents the number of edges in the linearized
      %                   friction cone
      %                   -- robot  A RigidBodyManipulator object. If we do
      %                   not want to solve an inverse kinematics problem,
      %                   leave this to empty. Default is empty
      if(nargin<6)
        options = struct();
      end
      obj = obj@ForceClosureContactsBase(A_xc,b_xc,num_contacts,mu_face,options);
      
      obj.epsilonG = epsilonG;
      
      % setup the linearized friction cone if it was used
      if(obj.lin_fc_flag)
        [obj,obj.fc_weights] = obj.newFree(obj.num_fc_edges,obj.num_contacts);
        
        for i = 1:obj.num_contacts
          [obj,obj.fc_QuatWeights{i}] = obj.newSym(10+obj.num_fc_edges);
          Quat_mask = tril(ones(4,4))~=0;
          obj = obj.addBilinearVariable([obj.fc_Quat{i}(Quat_mask);obj.fc_weights(:,i)],obj.fc_QuatWeights{i});
          % add Constraint that fc_rotmat{i}*fc_rotmat{i}' = eye(3)
          obj = obj.withEqs(reshape(replaceBilinearProduct(obj.fc_rotmat{i}*obj.fc_rotmat{i}',obj.fc_Quat{i}(Quat_mask),obj.fc_QuatWeights{i}(1:10,1:10)),3,3)-eye(3));
          % add constraint that
          % fc_weight(j,i)*(Quat(1,1)+Quat(2,2)+Quat(3,3)+Quat(4,4))=fc_weight(j,i);
          for j = 1:obj.num_fc_edges
            obj = obj.withEqs(obj.fc_QuatWeights{i}(1,10+j)+obj.fc_QuatWeights{i}(5,10+j)+obj.fc_QuatWeights{i}(8,10+j)+obj.fc_QuatWeights{i}(10,10+j)-obj.fc_weights(j,i));
          end
          
          % compute the friction force
          obj.fc_edges{i} = obj.fc_rotmat{i}*obj.fc_edges0;
          obj = obj.withEqs(obj.f(:,i)-replaceBilinearProduct(obj.fc_edges{i}*obj.fc_weights(:,i),[obj.fc_Quat{i}(Quat_mask);obj.fc_weights(:,i)],obj.fc_QuatWeights{i}));
        end
      end
      
      % constraint rank(G) = 6 by imposing GG'>=eps*eye(6);
      G = clean(graspTransferMatrix(obj.A_xc*obj.xc),1e-7);
      GG = zeros(6,6);
      Gf = zeros(6,1);
      for j = 1:obj.num_contacts
        GG = GG+reshape(replaceBilinearProduct(G(:,(j-1)*3+(1:3))*G(:,(j-1)*3+(1:3))',obj.xc(:,j),obj.XCCF{j}(1:3,1:3)),6,6);
              
        if(~obj.lin_fc_flag)
        % f'f > epsilon to guarantee that the force is in the interior of the cone
          f_norm_min = max((1/obj.num_contacts)^2,1);
          obj = obj.withPos(obj.XCCF{j}(7,7)+obj.XCCF{j}(8,8)+obj.XCCF{j}(9,9)-f_norm_min);
        end
        
        % compute Gf
        Wi = [obj.XCCF{j}(1:3,1:3) obj.XCCF{j}(1:3,7:9);obj.XCCF{j}(7:9,1:3) obj.XCCF{j}(7:9,7:9)];
        wi = [obj.xc(:,j);obj.f(:,j)];
        Gf = Gf+replaceBilinearProduct(G(:,3*(j-1)+(1:3))*obj.f(:,j),wi,Wi);
        
        if(obj.lin_fc_flag)
          % fc_weights>1/obj.num_fc_edges
          obj = obj.withPos(obj.fc_weights(:,j)-1/obj.num_fc_edges*ones(obj.num_fc_edges,1));
        else
          % friction cone constraint
          % |c'*f|>= 1/sqrt(1+mu^2)*|f|
          obj = obj.withLor([sqrt(obj.mu_face^2+1)*(obj.XCCF{j}(4,7)+obj.XCCF{j}(5,8)+obj.XCCF{j}(6,9));obj.f(:,j)]);
          % |c'*f|>= 1/sqrt(1+mu^2)*|c|*|f|
          cf = obj.XCCF{j}(4:6,7:9);
          obj = obj.withLor([sqrt(obj.mu_face^2+1)*(obj.XCCF{j}(4,7)+obj.XCCF{j}(5,8)+obj.XCCF{j}(6,9));cf(:)]);
  %         % |cross(c,f)|<=mu/sqrt(1+mu^2)*|f|
  %         cross_cf = [obj.XCCF{j}(5,9)-obj.XCCF{j}(6,8);...
  %                     obj.XCCF{j}(6,7)-obj.XCCF{j}(4,9);...
  %                     obj.XCCF{j}(4,8)-obj.XCCF{j}(5,7)];
  %         obj = obj.withRLor([obj.XCCF{j}(7,7)+obj.XCCF{j}(8,8)+obj.XCCF{j}(9,9);obj.mu_face^2/(1+obj.mu_face^2);cross_cf]);
          % rank 1 constraint
        end
      end
      % Gf = 0;
      obj = obj.withEqs(Gf);
      obj = obj.withPSD(GG-epsilonG*eye(6));
    end
      
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      % @retval sol  A structure that contains contact_pos,c,d,f,etc
      % @retval sol_bilinear  A structure containing the bilinear matrices
      [sol,sol_bilinear] = retrieveSolution@ForceClosureContactsBase(obj,solver_sol);
      sol.f = double(solver_sol.eval(obj.f));
      if(obj.lin_fc_flag)
        sol.fc_weights = double(solver_sol.eval(obj.fc_weights));
      end
      if(nargout>1)
        sol_bilinear.XCCF = cell(obj.num_contacts,1);
        if(obj.lin_fc_flag)
          sol_bilinear.fc_QuatWeights = cell(obj.num_contacts,1);
        end
        for i = 1:obj.num_contacts
          sol_bilinear.XCCF{i} = double(solver_sol.eval(obj.XCCF{i}));
          if(obj.lin_fc_flag)
            sol_bilinear.fc_QuatWeights{i} = double(solver_sol.eval(obj.fc_QuatWeights{i}));
          end
        end
      end
    end
    
    function plotSolution(obj,sol,sol_bilinear)
      % plot the robot configuration if with_ik_flag = true;
      plotSolution@ForceClosureContactsBase(obj,sol,sol_bilinear);
      force_scalar = det(obj.A_xc'*obj.A_xc)^(1/6)*0.5/min(sqrt(sum(sol.f.^2,1)));
      for i = 1:obj.num_contacts
        sol.fc{i}.plot(obj.use_lcmgl,force_scalar*norm(sol.f(:,i))*0.5,sprintf('fc%d',i));
      end
      if(obj.use_lcmgl)
        for i = 1:obj.num_contacts
          lcmgl_force = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,sprintf('force%d',i));
          lcmgl_force.glColor3f(1,0,0);
          lcmgl_force.drawVector(sol.contact_pos(:,i),sol.f(:,i)*force_scalar,0.01*force_scalar,0.04*force_scalar,0.04*force_scalar);
          lcmgl_force.switchBuffers();
        end
      else
        hold on;
        for k = 1:obj.num_contacts
          arrow_start = sol.contact_pos(:,k);
          arrow_end = arrow_start+sol.f(:,k)*force_scalar;
          arrow_force = [arrow_start arrow_end];
          arrow_radius = norm(sol.f(:,k))*force_scalar*0.1;
          arrow3d(arrow_force(1,:),arrow_force(2,:),arrow_force(3,:),0.7,arrow_radius,arrow_radius*2,[1,0,0]);
        end
        hold off;
      end
    end
    
  end
  
  methods(Access = protected)
    function obj = addXCC(obj)
      [obj,obj.f] = obj.newFree(3,obj.num_contacts);
      obj.XCCF = cell(obj.num_contacts,1);
      for j = 1:obj.num_contacts
        [obj,obj.XCCF{j}] = obj.newSym(9);
        obj.XCC{j} = obj.XCCF{j}(1:6,1:6);
        [obj,xccf_ind_j] = obj.addBilinearVariable([obj.xc(:,j);obj.c(:,j);obj.f(:,j)],obj.XCCF{j});
        obj.xc_ind(:,j) = xccf_ind_j(1:3);
        obj.c_ind(:,j) = xccf_ind_j(4:6);
      end
    end
  end
end

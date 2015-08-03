classdef SearchContactsLinFC < SearchContactsBase
  % search for contacts that satisfies the Q1 norm is lower bounded, using
  % linearized friction cone
  properties(SetAccess = protected)
    monomial0 % The monomial for Lagrangian multiplier, for condition b >= radius
    monomial1 % The monomial for Lagrangian on constraint 1-a'*inv(Qw)*a = 0
    monomial2 % The monomial for Lagrangian on constraint V_cell{i}(:,j)'a+b>=0
    
    V_cell % A obj.num_contacts x 1 cell, V_cell{i} is a 6 x num_fc_edges matrix, containing all the primitive wrenches for the i'th contact point
    
    XCQuat % A obj.num_contacts x 1 cell, XCQuat{i} is a 13 x 13 matrix, supposedly the bilinear matrix [obj.xc(:,i);tril(obj.fc_Quat{i})]*[obj.xc(:,i);tril(obj.fc_Quat{i})]'
    Quat_idx % A 10 x obj.num_contacts matrix, Quat_idx(:,i) is the index of tril(obj.fc_Quat{i}) in obj.w
    % The ellipse is paramaterized as w'*Qw*w<=(radius)^2
    radius % A scalar.
    L0_mat % The monomial matrix of the Lagrangian multiplier, for b >= radius
    L1_mat % The monomial matrix of the Lagrangian multiplier, for 1-a'*inv(Qw)*a = 0;
    L2_mat % A obj.num_contacts x obj.num_fc_edges cell, L2_mat{i,j} is the monomial matrix of the Lagrangian V_cell{i}(:,j)'*a+b>=0
    sos_V  % b-radius-l1*(1-a'*inv(Qw)*a)-l2*(v'a+b)
    
    %%%%%%%%%%%
    % The following properties are for searching Lagrangian and contact
    % position simultaneously
    V_cell_var % A num_contacts x 1 cell, V_cell_var{i} is V_cell, but they are decision variables now
    VL2  % A num_contacts x num_fc_edges cell, VL2{i,j} is supposed to be the bilinear matrix of [V_cell_var{i}(:,j);tril(L2_mat{i,j})]*[V_cell_var{i}(:,j);tril(L2_mat{i,j})]'
    V_cell_var_idx % A num_contacts x 1 cell, V_cell_var_idx{i}(:,j) is the index of V_cell_var{i}(:,j) in obj.w
    XCQuat_bilinear % A num_contacts x 1 cell, XCQuat_bilinear{i} is the bilinear matrix for XCQuat{i}(1:3,4:13)*XCQuat{i}(1:3,4:13)'
%     xhatR  % A num_contacts x 1 cell, xhatR{i} is a 3 x 3 matrix, xhatR{i} = hat(contact_pos(:,i))*rotmat(fc_quat(:,i))
%     xhatR_bilinear % A num_contacts x 1 cell, xhatR_bilinear is a 9 x 9 matrix, xhatR_biinear{i} is xhatR{i}(:)*xhatR{i}(:)'
  end
  
  methods
    function obj = SearchContactsLinFC(A_xc,b_xc,disturbance_pos,num_contacts,mu_face,num_fc_edges,Qw,options)
      % @param options    -- A structure
      %                      -- robot  A RigidBodyManipulator object
      if(nargin<8)
        options = struct();
      end
      if(~isfield(options,'robot'))
        options.robot = [];
      end
      options.lin_fc_flag = true;
      options.num_fc_edges = num_fc_edges;
      obj = obj@SearchContactsBase(A_xc,b_xc,disturbance_pos,num_contacts,mu_face,Qw,options);

      
      % setup the linearized friction cone
      obj.V_cell = cell(obj.num_contacts,1);
      obj.XCQuat = cell(obj.num_contacts,1);
      obj.Quat_idx = zeros(10,obj.num_contacts);
      
      G = graspTransferMatrix(obj.contact_pos - bsxfun(@times,obj.disturbance_pos,ones(1,obj.num_contacts)));
      
      obj.radius = msspoly('r',1);
      obj.monomial0 = monomials([obj.a_indet;obj.b_indet],0:1);
      [obj,obj.L0_mat] = obj.newSym(length(obj.monomial0));
      obj = obj.withSOS(obj.monomial0'*obj.L0_mat*obj.monomial0 - 1e-4);
      obj.monomial1 = monomials([obj.a_indet;obj.b_indet],0:1);
      [obj,obj.L1_mat] = obj.newSym(length(obj.monomial1));
      obj.monomial2 = monomials([obj.a_indet;obj.b_indet],0:1);
      obj.L2_mat = cell(obj.num_contacts,obj.num_fc_edges);
      
      for i = 1:obj.num_contacts
        [obj,obj.XCQuat{i}] = obj.newSym(13);
        tril_mask = tril(ones(4,4)) ~= 0;
        [obj,w_idx] = obj.addBilinearVariable([obj.xc(:,i);obj.fc_Quat{i}(tril_mask)],obj.XCQuat{i});
        obj.Quat_idx(:,i) = w_idx(4:end);
        % constrain that xc(1,i)(Quat(1,1)+Quat(2,2)+Quat(3,3)+Quat(4,4) =
        % xc(1,i))
        for j = 1:3
          obj = obj.withEqs(obj.XCQuat{i}(j,4)+obj.XCQuat{i}(j,8)+obj.XCQuat{i}(j,11)+obj.XCQuat{i}(j,13)-obj.xc(j,i));
        end
        obj = obj.withEqs(obj.XCC{i}(1:3,1:3)-obj.XCQuat{i}(1:3,1:3));
        % constrain that fc_rotmat{i}*fc_rotmat{i}' = eye(3)
        obj = obj.withEqs(reshape(replaceBilinearProduct(obj.fc_rotmat{i}*obj.fc_rotmat{i}',obj.fc_Quat{i}(tril_mask),obj.XCQuat{i}(4:13,4:13)),3,3)-eye(3));
        % compute the primitive wrenches
        obj.V_cell{i} = clean(reshape(replaceBilinearProduct(G(:,(i-1)*3+(1:3))*obj.fc_edges{i},[obj.xc(:,i);obj.fc_Quat{i}(tril_mask)],obj.XCQuat{i}),6,obj.num_fc_edges),1e-10);
        
        for j = 1:obj.num_fc_edges
          [obj,obj.L2_mat{i,j}] = obj.newSym(length(obj.monomial2));
        end
      end
    end
    
    function [solver_sol,info,solver_time] = findContactsGivenLagrangian(obj,L2_mat,radius,w0)
      if(nargin<5)
        w0 = {};
      else
        w0 = {w0};
      end
      if(obj.search_lagrangian_flag)
        error('The problem is formulated to search Lagrangian and contact simultaneously');
      end
      obj.sos_V = obj.monomial0'*obj.L0_mat*obj.monomial0*(obj.b_indet-radius)-obj.monomial1'*obj.L1_mat*obj.monomial1*(1-obj.a_indet'*inv(obj.Qw)*obj.a_indet);
      for i = 1:obj.num_contacts
        for j = 1:obj.num_fc_edges
          obj.sos_V = obj.sos_V-obj.monomial2'*L2_mat{i,j}*obj.monomial2*(obj.V_cell{i}(:,j)'*obj.a_indet+obj.b_indet);
        end
      end
      obj = obj.withSOS(obj.sos_V);
      [solver_sol,info,~,solver_time] = obj.optimize(w0{:});
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      [sol,sol_bilinear] = retrieveSolution@ForceClosureContactsBase(obj,solver_sol);
      sol.L0_mat = double(solver_sol.eval(obj.L0_mat));
      sol.L2_mat = cell(obj.num_contacts,obj.num_fc_edges);
      sol.VL2 = cell(obj.num_contacts,obj.num_fc_edges);
      sol_bilinear.V_cell_var = cell(obj.num_contacts,1);
      sol_bilinear.XCQuat = cell(obj.num_contacts,1);
      for i = 1:obj.num_contacts
        if(~isempty(obj.V_cell_var))
          sol.V_cell_var{i} = double(solver_sol.eval(obj.V_cell_var{i}));
        end
        sol_bilinear.XCQuat{i} = double(solver_sol.eval(obj.XCQuat{i}));
        for j = 1:obj.num_fc_edges
          sol.L2_mat{i,j} = double(solver_sol.eval(obj.L2_mat{i,j}));
          if(~isempty(obj.VL2))
            sol_bilinear.VL2{i,j} = double(solver_sol.eval(obj.VL2{i,j}));
          end
        end
      end
    end
    
    function obj = constructPosEllipseVWithBilinear(obj)
      % construct the BMI problem such that it searches for both Lagrangian
      % multiplier and the contact positions
      obj.search_lagrangian_flag = true;
      obj.V_cell_var = cell(obj.num_contacts,1);
      obj.V_cell_var_idx = cell(obj.num_contacts,1);
      obj.VL2 = cell(obj.num_contacts,obj.num_fc_edges);
      obj.XCQuat_bilinear = cell(obj.num_contacts,1);
%       obj.xhatR = cell(obj.num_contacts,1);
%       obj.xhatR_bilinear = cell(obj.num_contacts,1);
      obj.sos_V = obj.b_indet-obj.radius*obj.s-obj.monomial1'*obj.L1_mat*obj.monomial1*(obj.s^2-obj.a_indet'*inv(obj.Qw)*obj.a_indet);
      for i = 1:obj.num_contacts
        [obj,obj.V_cell_var{i}] = obj.newFree(6,obj.num_fc_edges);
        obj = obj.withEqs(obj.V_cell_var{i}-obj.V_cell{i});
        obj.V_cell_var_idx{i} = zeros(6,obj.num_fc_edges);
%         [obj,obj.xhatR{i}] = obj.newFree(3,3);
%         tril_mask = tril(ones(4,4))~=0;
%         obj = obj.withEqs(obj.xhatR{i}-reshape(replaceBilinearProduct(vectorToSkewSymmetric(obj.contact_pos(:,i))*obj.fc_rotmat{i},[obj.xc(:,i);obj.fc_Quat{i}(tril_mask)],obj.XCQuat{i}),3,3));
%         [obj,obj.xhatR_bilinear{i}] = obj.newSym(9);
%         obj = obj.addBilinearVariable(reshape(obj.xhatR{i},[],1),obj.xhatR_bilinear{i});
%         % xhat*R*R'*xhat' = xhat*xhat'
%         obj = obj.withEqs(replaceBilinearProduct(obj.xhatR{i}*obj.xhatR{i},reshape(obj.xhatR{i},[],1),obj.xhatR_bilinear{i})...
%           -replaceBilinearProduct(vectorToSkewSymmetric(obj.contact_pos(:,i))*vectorToSkewSymmetric(obj.contact_pos(:,i))',obj.xc(:,i),obj.XCC{i}(1:3,1:3)));
        [obj,obj.XCQuat_bilinear{i}] = obj.newSym(30);
        obj = obj.addBilinearVariable(reshape(obj.XCQuat{i}(1:3,4:13),[],1),obj.XCQuat_bilinear{i});
        for j = 1:obj.num_fc_edges
          tril_mask = tril(ones(size(obj.L2_mat{i,j})))~= 0;
          L2_ij_lower = obj.L2_mat{i,j}(tril_mask);
          [obj,obj.VL2{i,j}] = obj.newSym(length(L2_ij_lower)+6);
          [obj,w_idx] = obj.addBilinearVariable([obj.V_cell_var{i}(:,j);L2_ij_lower],obj.VL2{i,j});
          obj.V_cell_var_idx{i}(:,j) = w_idx(1:6);
          obj.sos_V = obj.sos_V-replaceBilinearProduct(obj.monomial2'*obj.L2_mat{i,j}*obj.monomial2*(obj.V_cell_var{i}(:,j)'*obj.a_indet+obj.b_indet),[obj.V_cell_var{i}(:,j);L2_ij_lower],obj.VL2{i,j});
          obj = obj.withPSD(obj.L2_mat{i,j});
          % V_cell_var{i}(1:3,j) is the unit length vector, the edge of the
          % friction cone.
          obj = obj.withEqs(obj.VL2{i,j}(1,1)+obj.VL2{i,j}(2,2)+obj.VL2{i,j}(3,3)-1);
          % V_cell_var{i}(1:3,j) and V_cell_var{4:6,j) are perpendicular to
          % each other
          obj = obj.withEqs(obj.VL2{i,j}(1,4)+obj.VL2{i,j}(2,5)+obj.VL2{i,j}(3,6));
          % VL2{i,j}(1:3,1:3) should be equal to
          % V_cell{i}(1:3,j)*V_cell{i}(1:3,j)'
          tril_mask = tril(ones(4,4))~=0;
          obj = obj.withEqs(obj.VL2{i,j}(1:3,1:3)-reshape(replaceBilinearProduct(obj.V_cell{i}(1:3,j)*obj.V_cell{i}(1:3,j)',obj.fc_Quat{i}(tril_mask),obj.XCQuat{i}(4:end,4:end)),3,3));
          % |V_cell_var{i}(4:6,j)|<=|contact_pos(:,i)|
          obj = obj.withPos(replaceBilinearProduct(obj.contact_pos(:,i)'*obj.contact_pos(:,i),obj.xc(:,i),obj.XCC{i}(1:3,1:3))-obj.VL2{i,j}(4,4)-obj.VL2{i,j}(5,5)-obj.VL2{i,j}(6,6));
          % VL2{i,j}(4:6,4:6) should be equal to
          % V_cell{i}(4:6,j)*V_cell{i}(4:6,j)'
          obj = obj.withEqs(obj.VL2{i,j}(4:6,4:6)-reshape(replaceBilinearProduct(obj.V_cell{i}(4:6,j)*obj.V_cell{i}(4:6,j)',reshape(obj.XCQuat{i}(1:3,4:13),[],1),obj.XCQuat_bilinear{i}),3,3));
          % hat(xc)*fc_rotmat*fc_rotmat'*hat(xc)' = hat(xc)*hat(xc)'
          hat_xc = vectorToSkewSymmetric(obj.xc(:,i));
          tril_mask = tril(ones(4,4))~=0;
          xcR = reshape(replaceBilinearProduct(hat_xc*obj.fc_rotmat{i},[obj.xc(:,i);obj.fc_Quat{i}(tril_mask)],obj.XCQuat{i}),3,3);
          obj = obj.withEqs(reshape(replaceBilinearProduct(xcR*xcR',reshape(obj.XCQuat{i}(1:3,4:13),[],1),obj.XCQuat_bilinear{i}),3,3)-...
            reshape(replaceBilinearProduct(hat_xc*hat_xc',obj.xc(:,i),obj.XCC{i}(1:3,1:3)),3,3));
%           obj = obj.withEqs(obj.VL2{i,j}(4:6,4:6)-reshape(replaceBilinearProduct(obj.xhatR{i}*obj.fc_edges0(:,j)*obj.fc_edges0(:,j)'*obj.xhatR{i}',reshape(obj.xhatR{i},[],1),obj.xhatR_bilinear{i}),3,3));
        end
      end
    end
    
    function [contact_pos,fc_edges,L1_mat,L2_mat,solver_sol,info,sol,sol_bilinear] = findContactPositionLagrangian(obj,rho,s,w)
      if(nargin<4)
        w = {};
      else
        w = {w};
      end
      if(~obj.search_lagrangian_flag)
        error('The problem is formulated to search contact position only');
      end
      obj.sos_V = msubs(obj.sos_V,[obj.radius;obj.s],[sqrt(rho);s]);
      obj = obj.withSOS(obj.sos_V);
      [solver_sol,info] = obj.optimize(w{:});
      fc_edges = cell(obj.num_contacts,1);
      L2_mat = cell(obj.num_contacts,obj.num_fc_edges);
      if(info == 1)
        contact_pos = double(solver_sol.eval(obj.contact_pos));
        L1_mat = double(solver_sol.eval(obj.L1_mat));
        for i = 1:obj.num_contacts
          fc_edges{i} = double(solver_sol.eval(obj.fc_edges{i}));
          for j = 1:obj.num_fc_edges
            L2_mat{i,j} = double(solver_sol.eval(obj.L2_mat{i,j}));
          end
        end
      else
        contact_pos = [];
        L1_mat = [];
      end
      [sol,sol_bilinear] = obj.retrieveSolution(solver_sol);
    end
    
  end
end
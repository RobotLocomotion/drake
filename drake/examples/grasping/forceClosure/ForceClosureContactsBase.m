classdef ForceClosureContactsBase < InverseKinematicsBMI
  properties(SetAccess = protected)
    num_contacts % The number of contact points
    mu_face      % The friction coefficient
    
    % The contact points are parameterized by A_xc*obj.xc(:,i)+b_xc, where
    % A_xc represents the scaling, and b_xc represents the shifting
    A_xc % A 3 x 3 matrix
    b_xc % A 3 x 1 matrix, this b_xc should be roughtly at the geometric center of the object
    xc % A 3 x obj.num_contacts matrix, obj.xc(:,i) is the decision variable for the i'th contact point
    contact_pos % A 3 x obj.num_contacts matrix. contact_pos(:,i) is the contact point for the i'th contact
    
    % To search over all faces of the polyhedron, we use the
    % separating hyperplane c'*x+d=0 between the contact point and the
    % polyhedron
    c % The unit normal vector of the separating hyperplane. Point outward from the polyhedron
    d % The scalar of the separating hyperplane.
    
    XCC % XCC is a num_contacts x 1 cell, XCC{i} is a 6 x 6 bilinear matrix, supposed to be wi*wi', where wi = [obj.xc(:,i);obj.c(:,i)]
    
    grasped_geometry % A GraspedGeometry object
    %%%%%%%%%%%
    % THE PROPERTIES BELOW ARE ONLY FOR ELLIPSOID GEOMETRIES TO BE GRASPED
    t_ellipsoid % A 1 x obj.num_contacts vector. This is used when constraining the normal direction c(:,i) has unit length, % xc'*A_ellipsoid'*c = min_eig_A*t_ellipsoid
    XCTellipsoid % A obj.num_contacts x 1 cell, XCTellipsoid{i} is supposed to be the bilinear matrix wi*wi', where wi = [obj.xc(:,i);obj.t_ellipsoid(i)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % THE PROPERTIES BELOW ARE FOR LINEARIZED FRICTION CONES
    % we need to find the rotation matrix that would rotate [0;0;1] to
    % obj.c(:,i), since the linearized friction edges are parameterized by
    % using normal vector [0;0;1]
    lin_fc_flag % A boolean variable, true if we are using linearized friction cone. Default is false
    fc_quat % A 4 x obj.num_contacts matrix. fc_quat(:,i) is the quaternion, that rotates [0;0;1] to obj.c(:,i)
    fc_Quat % A obj.num_contacts x 1 cell. fc_Quat{i} is the bilinear matrix fc_quat(:,i)*fc_quat(:,i)'
    num_fc_edges % A scalar, the number of edges in each linearized friction cone
    fc_edges % A obj.num_contacts x 1 cell, fc_edges{i} is a 3 x num_fc_edges. fc_edges0(:,i) is a unit length vector, representing the edge of the friction cone at the i'th contact point
    fc_edges0 % A 3 x obj.num_fc_edges matrix. fc_edges0(:,i) is the i'th edge of the friction cone, if the cone axis is [0;0;1];
    fc_rotmat  % A num_contacts x 1 cell, fc_rotmat{i} is the rotation matrix of fc_quat(:,i);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % THE PROPERTIES BELOW ARE FOR SOLVING INVERSE KINEMATICS PROBLEM
    with_ik_flag % A boolean, True if we are also going to solve an inverse kinematics problem. Default is false
    
    
    % THE FOLLOWING PROPERTIES ARE FOR GRASPING
    grasp_body_idx % A 1 x num_contacts vector. obj.robot.getBody(grasp_body_idx(i)) has a grasp point matching contact_pos(:,i)
    
    % THE PROPERTIES BELOW ARE FOR COLLISION AVOIDANCE BETWEEN GRASPING
    % BODIES AND OBJECT.
    % Collision avoidance is enforced by find a separating hyperplane
    % c_obj_link(:,i)'x+d_obj_link(:,i)=0 between the link i and the object. 
    % c_obj_link(:,i)'x+d_obj_link(:,i)>=0 for all x on the link i
    % c_obj_link(:,i)'x+d_obj_link(:,i)<=0 for all x on the object
    % |c_obj_link(:,i)|=1
    c_obj_link % A 3 x obj.num_bodies matrix. c_obj_link(:,i) is the normal of separating hyperplane between body i and the object
    d_obj_link % A 1 x obj.num_bodies vector. d_obj_link(i) is the intercept of the separating hyperplane between body i and the object
    c_bodyposQuat % A cell. c_bodypos{i} is a 16 x 16 matrix, supposed to be [c_obj_link(:,i);body_pos(:,i);tril(body_Quat{i})]*[c_obj_link(:,i);body_pos(:,i);tril(body_Quat{i})]'
    body_collision_pts % A cell. body_collision_pts{i} is the vertices of the collision geometry on link i
    
    % THE FOLLOWING PROPERTIES ARE FOR GRASPING WITHOUT SPECIFYING THE
    % GRASPING POINT ON THE FINGER
    body_grasp_pts % A 3 x obj.num_contacts matrix, body_grasp_pts(:,i) is the contact point on the finger, in body frame
    bodyQuat_pts % A obj.num_contacts x 1 cell, bodyQuat_pt{i} is a 13 x 13 bilinear matrix, supposed to be wi*wi', where wi = [tril(body_Quat{obj.grasp_body_idx(i)}) body_grasp_pt(:,i)];
    rotate_body_pts % A 3 x obj.num_contacts, rotate_body_pts(:,i) = rotateMatFromQ(obj.body_Quat{obj.grasp_body_idx(i)})*body_grasp_pts(:,i)
    bodypos_rotate_pts % A obj.num_contacts matrix, body_pos_rotate_pts{i} is a 6 x 6 bilinear matrix, supposed to be wi*wi', where wi = [obj.body_pos(:,obj.grasp_body_idx(i));rotate_body_pts(:,i)]
    num_body_grasp_vertices % A obj.num_contacts x 1 vector, num_body_grasp_vertices(i) is the number of vertices on the grasp face of obj.grasp_body_idx(i)
    body_grasp_vertices_weights % A obj.num_contacts x 1 cell, body_grasp_vertices_weights{i} is the weight on the vertices of the face
    body_grasp_QuatWeights % A num_contacts x 1 cell, body_grasp_QuatWeights{i} is the (10+num_body_grasp_vertices(i))*(10+num_body_grasp_vertices(i)) bilinear matrix, supposed to be wi*wi, where wi = [tril(body_Quat{obj.grasp_body_idx(i)});body_grasp_vertices_weights(:,i)]
  end
  
  properties(Access = protected)
    xc_ind % A 3 x num_contacts matrix, w(xc_ind(:,i)) is xc(:,i);
    c_ind % A 3 x num_contacts matrix, w(c_ind(:,i)) is c(:,i);
  end
  
  methods
    function obj = ForceClosureContactsBase(A_xc,b_xc,num_contacts,mu_face,options)
      % @param A_xc     A 3 X 3 matrix. Refer to obj.A_xc property
      % @param b_xc     A 3 x 1 matrix. Refer to obj.b_xc property
      % @param num_contacts   A scalar. The number of contact points
      % @param mu_face    A scalar. The friction coefficient
      % @param options    A structure
      %                   -- lin_fc_flag   If true, use linearized friction
      %                   cone, default is false
      %                   -- num_fc_edges  If lin_fc_flag = true, this
      %                   represents the number of edges in the linearized
      %                   friction cone
      %                   -- robot  A RigidBodyManipulator object. If we do
      %                   not want to solve an inverse kinematics problem,
      %                   leave this to empty. Default is empty
      if(nargin<5)
        options = struct();
      end
      if(~isfield(options,'lin_fc_flag'))
        options.lin_fc_flag = false;
      end
      if(~isfield(options,'num_fc_edges'))
        options.num_fc_edges = 0;
      end
      if(options.lin_fc_flag &&(numel(options.num_fc_edges)~=1 || options.num_fc_edges<1))
        error('options.num_fc_edges should be a positive integer');
      end
      if(~isfield(options,'robot'))
        options.robot = {};
      end
      obj = obj@InverseKinematicsBMI();
      obj.with_ik_flag = false;
      obj.lin_fc_flag = options.lin_fc_flag;
      obj.num_fc_edges = options.num_fc_edges;
      
      if(any(size(A_xc) ~= [3,3]))
        error('A_xc should be a 3 x 3 matrix');
      end
      if(any(size(b_xc) ~= [3,1]))
        error('b_xc should be a 3 x 1 vector');
      end

      obj.A_xc = A_xc;
      obj.b_xc = b_xc;
      
      if(numel(num_contacts) ~= 1 || num_contacts<1)
        error('num_contacts should be a positive integer');
      end
      obj.num_contacts = num_contacts;
      
      if(numel(mu_face) ~= 1 || mu_face<0)
        error('mu_face should be a non-negative scalar');
      end
      obj.mu_face = mu_face;
      
      [obj,obj.xc] = obj.newFree(3,obj.num_contacts);
      obj.xc_ind = zeros(3,obj.num_contacts);
      % contact_pos(:,i) = obj.A_xc*obj.xc(:,i)+obj.b_xc;
      obj.contact_pos = obj.A_xc*obj.xc+bsxfun(@times,obj.b_xc,ones(1,obj.num_contacts));
      
      [obj,obj.c] = obj.newFree(3,obj.num_contacts);
      [obj,obj.d] = obj.newFree(1,obj.num_contacts);
      obj.c_ind = zeros(3,obj.num_contacts);
      
      % setup the linearized friction cone if it was used
      if(obj.lin_fc_flag)
        [obj,obj.fc_quat] = obj.newFree(4,obj.num_contacts);
        obj.fc_Quat = cell(obj.num_contacts,1);
        
        % fc_edges0 is the unit length edge on the friction cone, whose
        % normal vector is [0;0;1];
        obj.fc_edges0 = linFCedges(obj.num_fc_edges,obj.mu_face);
        obj.fc_edges = cell(obj.num_contacts,1);
        obj.fc_rotmat = cell(obj.num_contacts,1);
        for i = 1:obj.num_contacts
          [obj,obj.fc_Quat{i}] = obj.newSym(4);
          obj = obj.addBilinearVariable(obj.fc_quat(:,i),obj.fc_Quat{i});
          obj = obj.withEqs(obj.fc_Quat{i}(1,1)+obj.fc_Quat{i}(2,2)+obj.fc_Quat{i}(3,3)+obj.fc_Quat{i}(4,4)-1);
          obj.fc_rotmat{i} = rotmatFromQuatBilinear(obj.fc_Quat{i});
          obj = obj.withEqs(obj.fc_rotmat{i}*[0;0;1]-obj.c(:,i));
          % compute the edges of the friction cone 
          obj.fc_edges{i} = obj.fc_rotmat{i}*obj.fc_edges0;
        end
      end
      
      obj.XCC = cell(obj.num_contacts,1);
      obj = obj.addXCC();
      for j = 1:obj.num_contacts
        % c(:,j) is a unit vector
        obj = obj.withEqs(obj.XCC{j}(4,4)+obj.XCC{j}(5,5)+obj.XCC{j}(6,6)-1);
        % The contact point is on the separating hyperplane c'x+d = 0
        obj = obj.withEqs(clean(replaceBilinearProduct(obj.c(:,j)'*obj.contact_pos(:,j),[obj.xc(:,j);obj.c(:,j)],obj.XCC{j}),1e-7)+obj.d(j));
      end
      if(~isempty(options.robot))
        obj = obj.addRobot(options.robot);
      end
    end
    
    function obj = addRobot(obj,robot)
      % calling this function would add the kinematics of the robot to the
      % combined problem
      obj = addRobot@InverseKinematicsBMI(obj,robot);
      obj.with_ik_flag = true;
      
      
      % Add the collision avoidance between the object and robot
      obj.c_obj_link = msspoly.zeros(3,obj.num_bodies);
      obj.d_obj_link = msspoly.zeros(1,obj.num_bodies);
      obj.c_bodyposQuat = cell(obj.num_bodies,1);
      obj.body_collision_pts = cell(obj.num_bodies,1);
    end
    
    function obj = assignGraspBody(obj,grasp_body_idx)
      % assign grasp_body_idx(i) to be the body that has a grasp point matching contact_pos(:,i)
      % @param grasp_body_idx   A 1 x obj.num_contacts vector
      sizecheck(grasp_body_idx,[1,obj.num_contacts]);
      obj.grasp_body_idx = grasp_body_idx;
      for i = 1:obj.num_contacts
        obj.c_obj_link(:,grasp_body_idx(i)) = obj.c(:,i);
        obj.d_obj_link(:,grasp_body_idx(i)) = obj.d(i);
        % forwardKin position matches with contact location
        body_i_rotmat = rotmatFromQuatBilinear(obj.body_Quat{grasp_body_idx(i)});
        % constrain that the grasp body is within the halfspace
        % c'x+d>0
        body_collision_geometry = obj.robot.getBody(grasp_body_idx(i)).getCollisionGeometry();
        if(length(body_collision_geometry) ~= 1 || ~isa(body_collision_geometry{1},'RigidBodyBox'))
          error('Not implemented, it should have one RigidBodyBox collision geometry');
        end
        obj.body_collision_pts{grasp_body_idx(i)} = body_collision_geometry{1}.getTerrainContactPoints;
        num_collision_pts = size(obj.body_collision_pts{grasp_body_idx(i)},2);
        [obj,obj.c_bodyposQuat{obj.grasp_body_idx(i)}] = obj.newSym(16);
        Quat_tril_mask = tril(ones(4,4))~=0;
        Quat_tril = obj.body_Quat{obj.grasp_body_idx(i)}(Quat_tril_mask);
        obj = obj.addBilinearVariable([obj.c(:,i);obj.body_pos(:,obj.grasp_body_idx(i));Quat_tril],obj.c_bodyposQuat{obj.grasp_body_idx(i)});
        obj = obj.withEqs(obj.XCC{i}(4:6,4:6)-obj.c_bodyposQuat{obj.grasp_body_idx(i)}(1:3,1:3));
        
        % body_i_rotmat*body_i_rotmat' = eye(3);
        obj = obj.withEqs(replaceBilinearProduct(body_i_rotmat*body_i_rotmat',Quat_tril,obj.c_bodyposQuat{obj.grasp_body_idx(i)}(7:16,7:16))-eye(3));
        % c(j,i)*(Quat(1,1)+Quat(2,2)+Quat(3,3)+Quat(4,4)) = c(j,i)
        for j = 1:3
          obj = obj.withEqs(obj.c_bodyposQuat{obj.grasp_body_idx(i)}(j,7)+obj.c_bodyposQuat{obj.grasp_body_idx(i)}(j,11)+obj.c_bodyposQuat{obj.grasp_body_idx(i)}(j,14)+obj.c_bodyposQuat{obj.grasp_body_idx(i)}(j,16)-obj.c(j,i));
        end
        % c'x+d>=0 for all x being the vertices of body
        obj = obj.withPos(replaceBilinearProduct(obj.c(:,i)'*(repmat(obj.body_pos(:,obj.grasp_body_idx(:,i)),1,num_collision_pts)...
          +body_i_rotmat*obj.body_collision_pts{obj.grasp_body_idx(i)}),[obj.c(:,i);obj.body_pos(:,obj.grasp_body_idx(:,i));Quat_tril],obj.c_bodyposQuat{obj.grasp_body_idx(i)})+obj.d(i)*ones(1,num_collision_pts));
      end
    end
    
    function obj = assignGraspFingerPointNormal(obj,grasp_body_idx,grasp_pts,grasp_pts_normal)
      % assign point grasp_pts(:,i) on body grasp_body_idx(i) to be
      % the i'th contact points, with contact position obj.xc(:,i)
      % @param grasp_body_idx   a 1 x obj.num_contacts vector.
      % grasp_body_idx(i) is the index of the i'th grasping body
      % @param grasp_pts   A 3 x obj.num_contacts matrix. grasp_pts(:,i) is
      % the location of the i'th contact point in the body frame
      % @param grasp_body_normal  A 3 x obj.num_contacts matrix,
      % grasp_pts_normal(:,i) is the unit normal vector (pointing outward) on
      % the body frame, at grasp_pts(:,i)
      obj = assignGraspBody(obj,grasp_body_idx);
      sizecheck(grasp_pts,[3,obj.num_contacts]);
      if(any(size(grasp_pts_normal)~=[3,obj.num_contacts]) || any(abs(sum(grasp_pts_normal.^2,1)-ones(1,obj.num_contacts))>1e-3))
        error('grasp_pts_normal should be a 3 x obj.num_contacts matrix, each column being a unit vector');
      end
      for i = 1:obj.num_contacts
        % forwardKin position matches with contact location
        body_i_rotmat = rotmatFromQuatBilinear(obj.body_Quat{obj.grasp_body_idx(i)});
        contact_pos_body = body_i_rotmat*grasp_pts(:,i)+obj.body_pos(:,obj.grasp_body_idx(i));
        obj = obj.withEqs(contact_pos_body-obj.contact_pos(:,i));
        Quat_tril_mask = tril(ones(4,4))~=0;
        Quat_tril = obj.body_Quat{obj.grasp_body_idx(i)}(Quat_tril_mask);
        obj = obj.withEqs(replaceBilinearProduct(contact_pos_body*contact_pos_body',[obj.body_pos(:,obj.grasp_body_idx(i));Quat_tril],obj.c_bodyposQuat{obj.grasp_body_idx(i)}(4:16,4:16))-...
          replaceBilinearProduct(obj.contact_pos(:,i)*obj.contact_pos(:,i)',obj.xc(:,i),obj.XCC{i}(1:3,1:3)));
        % c'x+d <= epsilon for x being in the world position of grasp_pts(:,i)
        Quat_tril_mask = tril(ones(4,4))~=0;
        Quat_tril = obj.body_Quat{obj.grasp_body_idx(i)}(Quat_tril_mask);
        obj = obj.withPos(1e-5-(replaceBilinearProduct(obj.c(:,i)'*(obj.body_pos(:,grasp_body_idx(i))+body_i_rotmat*grasp_pts(:,i)),...
          [obj.c(:,i);obj.body_pos(:,grasp_body_idx(i));Quat_tril;],obj.c_bodyposQuat{obj.grasp_body_idx(i)})+obj.d(i)));
        % The normal vector of the body is in the opposite direction of the
        % normal vector on the object
        obj = obj.withEqs(body_i_rotmat*grasp_pts_normal(:,i)+obj.c(:,i));
      end
    end
    
    function obj = assignGraspFingerFaceNormal(obj,grasp_body_idx,body_face_vertices,body_face_normal)
      % assign a face on link grasp_body_idx{i} to be the contact faces on
      % the robot.
      % @param grasp_body_idx a 1 x obj.num_contacts vector.
      % grasp_body_idx(i) is the index of the i'th grasping body
      % @param body_face_vertices  a obj.num_contacts x 1 cell,
      % body_face_vertices{i} is a 3 x N matrix, with each column as the
      % vertex of the grasping face
      % @param body_face_normal  A 3 x obj.num_contacts matrix,
      % body_face_normal(:,i) is the outer normal vector for the grasp face
      % on obj.grasp_body_idx(i)
      obj = assignGraspBody(obj,grasp_body_idx);
      if(~iscell(body_face_vertices) || numel(body_face_vertices)~=obj.num_contacts)
        error('body_face_vertices should be a %d x 1 cell',obj.num_contacts);
      end
      if(any(size(body_face_normal)~=[3,obj.num_contacts]) || any(abs(sum(body_face_normal.^2,1)-ones(1,obj.num_contacts))>1e-3))
        error('body_face_normal should be 3 x obj.num_contacts matrix with unit length columns');
      end
      [obj,obj.body_grasp_pts] = obj.newFree(3,obj.num_contacts);
      [obj,obj.rotate_body_pts] = obj.newFree(3,obj.num_contacts);
      obj.bodyQuat_pts = cell(obj.num_contacts,1);
      obj.bodypos_rotate_pts = cell(obj.num_contacts,1);
      obj.num_body_grasp_vertices = zeros(obj.num_contacts,1);
      obj.body_grasp_vertices_weights = cell(obj.num_contacts,1);
      obj.body_grasp_QuatWeights = cell(obj.num_contacts,1);
      for i = 1:obj.num_contacts
        if(size(body_face_vertices{i},1)~=3)
          error('body_face_vertices{%d} should have 3 rows');
        end
        obj.num_body_grasp_vertices(i) = size(body_face_vertices{i},2);
        % constrain the body grasp point to be on the desired face
        [A_face,b_face,Aeq_face,beq_face] = vert2lcon(body_face_vertices{i}');
        obj = obj.withPos(b_face-A_face*obj.body_grasp_pts(:,i));
        if(~isempty(Aeq_face))
          obj = obj.withEqs(beq_face-Aeq_face*obj.body_grasp_pts(:,i));
        end
        % compute the contact point in the world frame
        bodyi_rotmat = rotmatFromQuatBilinear(obj.body_Quat{obj.grasp_body_idx(i)});
        Quat_tril_mask = tril(ones(4,4)~=0);
        Quat_tril = obj.body_Quat{obj.grasp_body_idx(i)}(Quat_tril_mask);
        [obj,obj.bodyQuat_pts{i}] = obj.newSym(13);
        wi1 = [Quat_tril;obj.body_grasp_pts(:,i)];
        obj = obj.addBilinearVariable(wi1,obj.bodyQuat_pts{i});
        obj = obj.withEqs(obj.bodyQuat_pts{i}(1:10,1:10)-obj.c_bodyposQuat{obj.grasp_body_idx(i)}(7:16,7:16));
        obj = obj.withEqs(obj.rotate_body_pts(:,i)-replaceBilinearProduct(bodyi_rotmat*obj.body_grasp_pts(:,i),wi1,obj.bodyQuat_pts{i}));
        % body_grasp_pts(j,i)*(body_Quat{i}(1,1)+body_Quat{i}(2,2)+body_Quat{i}(3,3)+body_Quat{4,4})
        % = body_grasp_pts(j,i);
        for j = 1:3
          obj = obj.withEqs(obj.bodyQuat_pts{i}(j+10,1)+obj.bodyQuat_pts{i}(j+10,5)+obj.bodyQuat_pts{i}(j+10,8)+obj.bodyQuat_pts{i}(j+10,10)-obj.body_grasp_pts(j,i));
        end
        % introduce the vertices weights
        [obj,obj.body_grasp_vertices_weights{i}] = obj.newFree(obj.num_body_grasp_vertices(i),1);
        obj = obj.withPos(obj.body_grasp_vertices_weights{i});
        obj = obj.withEqs(sum(obj.body_grasp_vertices_weights{i})-1);
        % body_pt =
        % body_face_vertices{i}(:,j)*body_grasp_vertices_weights{i}(j)
        body_pt = msspoly.zeros(3,1);
        % rotate_grasp_pt =
        % bodyi_rotmat*body_face_vertices{i}(:,j)*body_grasp_vertices_weights{i}(j)
        rotate_grasp_pt = msspoly.zeros(3,1);
        for j = 1:obj.num_body_grasp_vertices(i)
          body_pt = body_pt+obj.body_grasp_vertices_weights{i}(j)*body_face_vertices{i}(:,j);
          rotate_grasp_pt = rotate_grasp_pt+bodyi_rotmat*body_face_vertices{i}(:,j)*obj.body_grasp_vertices_weights{i}(j);
        end
        obj = obj.withEqs(obj.body_grasp_pts(:,i)-body_pt);
        
        [obj,obj.body_grasp_QuatWeights{i}] = obj.newSym(10+obj.num_body_grasp_vertices(i));
        wi3 = [Quat_tril;obj.body_grasp_vertices_weights{i}];
        obj = obj.addBilinearVariable(wi3,obj.body_grasp_QuatWeights{i});
        obj = obj.withEqs(obj.body_grasp_QuatWeights{i}(1:10,1:10)-obj.c_bodyposQuat{obj.grasp_body_idx(i)}(7:16,7:16));
        % Quat*(weight1+...+weightN) = Quat
        obj = obj.withEqs(sum(obj.body_grasp_QuatWeights{i}(11:end,1:10),1)'-Quat_tril);
        % (weight1+...+weightN)^2 = 1
        obj = obj.withEqs(replaceBilinearProduct(sum(obj.body_grasp_vertices_weights{i})^2,obj.body_grasp_vertices_weights{i},obj.body_grasp_QuatWeights{i}(11:end,11:end))-1);
        % weigh1^2+...+weightN^2>=1/N
        obj = obj.withPos(sum(diag(obj.body_grasp_QuatWeights{i}(11:end,11:end)))-1/obj.num_body_grasp_vertices(i));
        % rotmat*(vert1*weight1+...+vertN*weightN) =
        % obj.rotate_body_pts(:,i)
        obj = obj.withEqs(replaceBilinearProduct(rotate_grasp_pt,wi3,obj.body_grasp_QuatWeights{i})-obj.rotate_body_pts(:,i));
        % (vert1*weight1+...+vertN*weightN)*(vert1*weight1+...+vertN*weightN)'
        % = body_grasp_pts(:,i)*body_grasp_pts(:,i)'
        obj = obj.withEqs(replaceBilinearProduct(body_pt*body_pt',obj.body_grasp_vertices_weights{i},obj.body_grasp_QuatWeights{i}(11:end,11:end))-obj.bodyQuat_pts{i}(11:13,11:13));
        
        % The contact points computed from the body forward kinematics
        % matches with the contact_pos
        bodyi_grasp_pt_world = obj.body_pos(:,obj.grasp_body_idx(i))+obj.rotate_body_pts(:,i);
        obj = obj.withEqs(bodyi_grasp_pt_world-obj.contact_pos(:,i));
        
        [obj,obj.bodypos_rotate_pts{i}] = obj.newSym(6);
        obj = obj.withEqs(obj.bodypos_rotate_pts{i}(1:3,1:3)-obj.c_bodyposQuat{obj.grasp_body_idx(i)}(4:6,4:6));
        % body_grasp_pts'*rotmat'*rotmat*body_grasp_pts =
        % body_grasp_pts'*body_grasp_pts
        obj = obj.withEqs(replaceBilinearProduct(obj.rotate_body_pts(:,i)'*obj.rotate_body_pts(:,i),obj.rotate_body_pts(:,i),obj.bodypos_rotate_pts{i}(4:6,4:6))-...
          replaceBilinearProduct(obj.body_grasp_pts(:,i)'*obj.body_grasp_pts(:,i),obj.body_grasp_pts(:,i),obj.bodyQuat_pts{i}(11:13,11:13)));
        wi2 = [obj.body_pos(:,obj.grasp_body_idx(i));obj.rotate_body_pts(:,i)];
        obj = obj.addBilinearVariable(wi2,obj.bodypos_rotate_pts{i});
        % contact_pos*contact_pos' =
        % (body_pos+rotmat*body_grasp_pts)*(body_pos+rotmat*body_grasp_pts)'
        obj = obj.withEqs(replaceBilinearProduct(bodyi_grasp_pt_world*bodyi_grasp_pt_world',wi2,obj.bodypos_rotate_pts{i})-replaceBilinearProduct(obj.contact_pos(:,i)*obj.contact_pos(:,i)',obj.xc(:,i),obj.XCC{i}(1:3,1:3)));
        % The normal vector of the body is in the opposite direction of the
        % normal vector on the object
        obj = obj.withEqs(bodyi_rotmat*body_face_normal(:,i)+obj.c(:,i));
      end
    end
    
    function obj = addObject2LinkCollisionAvoidance(obj,link_idx,margin)
      % add a seperating hyperplane between the grasped object and robot
      % link, with distance at least equal to 'margin'
      % @param link_idx    The index of the link
      % @param margin      A non-negative scalar. The minimum distance is
      % at least 'margin'
      if(numel(link_idx)~=1)
        error('link_idx should be a scalar');
      end
      if(numel(margin)~= 1)
        error('margin should be a scalar');
      end
      [obj,obj.c_obj_link(:,link_idx)] = obj.newFree(3,1);
      [obj,obj.d_obj_link(link_idx)] = obj.newFree(1,1);
      [obj,obj.c_bodyposQuat{link_idx}] = obj.newSym(16);
      
      Quat_tril_mask = tril(ones(4,4))~=0;
      Quat_tril = obj.body_Quat{link_idx}(Quat_tril_mask);
      obj = obj.addBilinearVariable([obj.c_obj_link(:,link_idx);obj.body_pos(:,link_idx);Quat_tril],obj.c_bodyposQuat{link_idx});
      
      % c being unit vector
      obj = obj.withEqs(obj.c_bodyposQuat{link_idx}(1,1)+obj.c_bodyposQuat{link_idx}(2,2)+obj.c_bodyposQuat{link_idx}(3,3)-1);
      
      % grasped object in the halfspace
      % c_obj_link(:,link_idx)'*x+d_obj_link(link_idx)+margin<=0
      obj = obj.objectInHalfspace(obj.c_obj_link(:,link_idx),obj.d_obj_link(link_idx),margin);
      
      % robot link in the halfspace
      % c_obj_link(:,link_idx)'*x+d_obj_link(link_idx)>=0
      body_rotmat = rotmatFromQuatBilinear(obj.body_Quat{link_idx});
      % body_rotmat*body_rotmat' = I
      obj = obj.withEqs(replaceBilinearProduct(body_rotmat*body_rotmat',obj.body_Quat{link_idx}(Quat_tril_mask),obj.c_bodyposQuat{link_idx}(7:16,7:16))-eye(3));
      
      body_collision_geometry = obj.robot.getBody(link_idx).getCollisionGeometry();
      if(length(body_collision_geometry) ~= 1 || ~isa(body_collision_geometry{1},'RigidBodyBox'))
        error('Not implemented, it should have one RigidBodyBox collision geometry');
      end
      obj.body_collision_pts{link_idx} = body_collision_geometry{1}.getTerrainContactPoints;
      num_collision_pts = size(obj.body_collision_pts{link_idx},2);
      % c_obj_link'*(body_pos(:,link)+rotmat*body_collision_pts)+d_obj_link>=0
      obj = obj.withPos(replaceBilinearProduct(obj.c_obj_link(:,link_idx)'*(repmat(obj.body_pos(:,link_idx),1,num_collision_pts)+...
        body_rotmat*obj.body_collision_pts{link_idx}),[obj.c_obj_link(:,link_idx);obj.body_pos(:,link_idx);obj.body_Quat{link_idx}(Quat_tril_mask)],obj.c_bodyposQuat{link_idx})+...
        obj.d_obj_link(link_idx)*ones(1,num_collision_pts));
    end
    
    function [c_obj_link_sol,d_obj_link_sol,c_bodyposQuat_sol] = retrieveCollisionAvoidanceSolution(obj,solver_sol)
      c_obj_link_sol = double(solver_sol.eval(obj.c_obj_link));
      d_obj_link_sol = double(solver_sol.eval(obj.d_obj_link));
      c_bodyposQuat_sol = cell(obj.num_bodies,1);
      
      for i = 1:obj.num_bodies
        c_bodyposQuat_sol{i} = double(solver_sol.eval(obj.c_bodyposQuat{i}));
      end
    end
    
    function [sol,sol_bilinear] = retrieveSolution(obj,solver_sol)
      [sol,sol_bilinear] = retrieveSolution@InverseKinematicsBMI(obj,solver_sol);
      sol.xc = double(solver_sol.eval(obj.xc));
      sol.contact_pos = double(solver_sol.eval(obj.contact_pos));
      sol.c = double(solver_sol.eval(obj.c));
      sol.d = double(solver_sol.eval(obj.d));
      if(obj.lin_fc_flag)
        sol.fc_edges = cell(obj.num_contacts,1);
        sol.fc_rotmat = cell(obj.num_contacts,1);
        for i = 1:obj.num_contacts
          sol.fc_edges{i} = double(solver_sol.eval(obj.fc_edges{i}));
          sol.fc_rotmat{i} = double(solver_sol.eval(obj.fc_rotmat{i}));
        end
      end
      sol.fc = cell(obj.num_contacts,1);
      if(obj.lin_fc_flag)
        for i = 1:obj.num_contacts
          sol.fc{i} = LinearizedFrictionCone(sol.contact_pos(:,i),sol.c(:,i),obj.mu_face,sol.fc_edges{i});
        end
      else
        for i = 1:obj.num_contacts
          sol.fc{i} = FrictionCone(sol.contact_pos(:,i),sol.c(:,i),obj.mu_face);
        end
      end
      if(nargout>1)
        sol_bilinear.XCC = cell(obj.num_contacts,1);
        if(obj.lin_fc_flag)
          sol_bilinear.fc_Quat = cell(obj.num_contacts,1);
        end
        for i = 1:obj.num_contacts
          sol_bilinear.XCC{i} = double(solver_sol.eval(obj.XCC{i}));
          if(obj.lin_fc_flag)
            sol_bilinear.fc_Quat{i} = double(solver_sol.eval(obj.fc_Quat{i}));
          end
        end
      end
      if(obj.with_ik_flag)
        if(~isempty(obj.body_grasp_pts))
          sol.body_grasp_pts = double(solver_sol.eval(obj.body_grasp_pts));
          sol.rotate_body_pts = double(solver_sol.eval(obj.rotate_body_pts));
          sol.body_grasp_vertices_weights = cell(obj.num_contacts,1);
          for i = 1:obj.num_contacts
            sol.body_grasp_vertices_weights{i} = double(solver_sol.eval(obj.body_grasp_vertices_weights{i}));
          end
        end
        if(nargout > 1)
          [sol.c_obj_link,sol.d_obj_link,sol_bilinear.c_bodyposQuat] = obj.retrieveCollisionAvoidanceSolution(solver_sol);
          if(~isempty(obj.body_grasp_pts))
            sol_bilinear.bodyQuat_pts = cell(obj.num_contacts,1);
            sol_bilinear.bodypos_rotate_pts = cell(obj.num_contacts,1);
            sol_bilinear.body_grasp_QuatWeights = cell(obj.num_contacts,1);
            for i = 1:obj.num_contacts
              sol_bilinear.bodyQuat_pts{i} = double(solver_sol.eval(obj.bodyQuat_pts{i}));
              sol_bilinear.bodypos_rotate_pts{i} = double(solver_sol.eval(obj.bodypos_rotate_pts{i}));
              sol_bilinear.body_grasp_QuatWeights{i} = double(solver_sol.eval(obj.body_grasp_QuatWeights{i}));
            end
          end
        end
      end
    end
    
    function plotSolution(obj,sol,sol_bilinear)
      plotSolution@InverseKinematicsBMI(obj,sol,sol_bilinear);
      if(~obj.use_lcmgl)
        camlight
        lighting phong
      end
      obj.grasped_geometry.plotGeometry(obj.use_lcmgl);
    end
    
    function obj = setGraspedGeometry(obj,grasped_geometry)
      if(~isa(grasped_geometry,'GraspedGeometry'))
        error('grasped_geometry should be a GraspedGeometry object');
      end
      obj.grasped_geometry = grasped_geometry;
      if(obj.grasped_geometry.type == GraspedGeometry.POLYHEDRON_TYPE)
        obj = obj.setGraspedPolyhedron();
      elseif(obj.grasped_geometry.type == GraspedGeometry.SPHERE_TYPE)
        obj = obj.setGraspedSphere();
      elseif(obj.grasped_geometry.type == GraspedGeometry.ELLIPSOID_TYPE)
        obj = obj.setGraspedEllipsoid();
      elseif(obj.grasped_geometry.type == GraspedGeometry.CYLINDER_TYPE)
        obj = obj.setGraspedCylinder();
      else
        error('Unsupported GraspedGeometry');
      end
    end
    
    function obj = objectInHalfspace(obj,c_plane,d_plane,margin)
      % This method would add the constraint such that the grasped object is within the halfspace c_plane'*x+d_plane<=-margin, with the assumption that |c_plane| = 1
      if(obj.grasped_geometry.type == GraspedGeometry.POLYHEDRON_TYPE)
        obj = obj.withPos(-(c_plane'*obj.grasped_geometry.Pobj.V'+d_plane*ones(1,size(obj.grasped_geometry.Pobj.V,1))+margin*ones(1,size(obj.grasped_geometry.Pobj.V,1))));
      elseif(obj.grasped_geometry.type == GraspedGeometry.SPHERE_TYPE)
        obj = obj.withPos(-(c_plane'*obj.grasped_geometry.sphere_center+d_plane+margin)-obj.grasped_geometry.sphere_radius);
      elseif(obj.grasped_geometry.type == GraspedGeometry.ELLIPSOID_TYPE)
        % c_plane'*(A_ellipsoid*x+b_ellipsoid)+d_plane+margin<=0 for all |x|<=1
        obj = obj.withLor([-(c_plane'*obj.grasped_geometry.b_ellipsoid+d_plane+margin);obj.grasped_geometry.A_ellipsoid'*c_plane]);
      elseif(obj.grasped_geometry.type == GraspedGeometry.CYLINDER_TYPE)
        %c_plane*(R_cylinder*[r_cylinder*x;r_cylinder*y;h_cylinder*z]+b_cylinder)+d_plane+margin<=0
        %for all x^2+y2=1 and z=1 or -1
        cA = c_plane'*obj.grasped_geometry.R_cylinder*diag([obj.grasped_geometry.r_cylinder;obj.grasped_geometry.r_cylinder;obj.grasped_geometry.h_cylinder]);
        obj = obj.withLor([-(cA(3)+c_plane'*obj.grasped_geometry.b_cylinder+d_plane+margin);cA(1:2)']);
        obj = obj.withLor([-(-cA(3)+c_plane'*obj.grasped_geometry.b_cylinder+d_plane+margin);cA(1:2)']);
      else
        error('Unsupported GraspedGeometry');
      end
    end
  end
  
  methods(Access = protected)
    function obj = setGraspedPolyhedron(obj)
      % Constrain the contact points to be within the shrunk polygon
      for k = 1:obj.num_contacts
        obj = obj.withPos(obj.grasped_geometry.bin_shrunk - obj.grasped_geometry.Ain_shrunk*obj.contact_pos(:,k));
      end
      
      for j = 1:obj.num_contacts
        % All the vertices of the polyhedron satisfies c'x+d<0
        obj = obj.withPos(-(obj.c(:,j)'*obj.grasped_geometry.verts + obj.d(j)*ones(1,obj.grasped_geometry.num_verts)));

        % Contact points must be outside of the inner sphere
        obj = obj.withPos(obj.XCC{j}(1,1)+obj.XCC{j}(2,2)+obj.XCC{j}(3,3)-1);
      end
    end
    
    function obj = setGraspedSphere(obj)
      for i = 1:obj.num_contacts
        % constrain the contact points to be on the sphere
        obj = obj.withEqs(obj.XCC{i}(1,1)+obj.XCC{i}(2,2)+obj.XCC{i}(3,3)-1);
        % constrain the normal vector
        obj = obj.withEqs(obj.c(:,i)-obj.xc(:,i));
        obj = obj.withEqs(obj.XCC{i}(1:3,1:3)-obj.XCC{i}(4:6,4:6));
        obj = obj.withEqs(obj.XCC{i}(1:3,1:3)-obj.XCC{i}(4:6,1:3));
      end
    end
    
    function obj = setGraspedEllipsoid(obj)
      % compute some sample points on the ellipsoid
      [x_sphere,y_sphere,z_sphere] = sphere(20);
      x_ellipsoid = obj.grasped_geometry.A_ellipsoid(1,1)*x_sphere+obj.grasped_geometry.A_ellipsoid(1,2)*y_sphere+obj.grasped_geometry.A_ellipsoid(1,3)*z_sphere+obj.grasped_geometry.b_ellipsoid(1);
      y_ellipsoid = obj.grasped_geometry.A_ellipsoid(2,1)*x_sphere+obj.grasped_geometry.A_ellipsoid(2,2)*y_sphere+obj.grasped_geometry.A_ellipsoid(2,3)*z_sphere+obj.grasped_geometry.b_ellipsoid(2);
      z_ellipsoid = obj.grasped_geometry.A_ellipsoid(3,1)*x_sphere+obj.grasped_geometry.A_ellipsoid(3,2)*y_sphere+obj.grasped_geometry.A_ellipsoid(3,3)*z_sphere+obj.grasped_geometry.b_ellipsoid(3);
      ellipsoid_sample = [x_ellipsoid(:)';y_ellipsoid(:)';z_ellipsoid(:)'];
      
      [obj,obj.t_ellipsoid] = obj.newFree(1,obj.num_contacts);
      obj.XCTellipsoid = cell(obj.num_contacts,1);
      for i = 1:obj.num_contacts
        % constrain the contact points to be on the ellipsoid
        obj = obj.withEqs(obj.XCC{i}(1,1)+obj.XCC{i}(2,2)+obj.XCC{i}(3,3)-1);
        % c being the normal direction on the ellipsoid means A_ellipsoid'*c =
        % min_eig_A*t*xc
        % where t is a positive scalar
        [obj,obj.XCTellipsoid{i}] = obj.newSym(4);
        obj = obj.addBilinearVariable([obj.xc(:,i);obj.t_ellipsoid(i)],obj.XCTellipsoid{i});
        obj = obj.withEqs(obj.XCTellipsoid{i}(1:3,1:3)-obj.XCC{i}(1:3,1:3));
        obj = obj.withPos(obj.t_ellipsoid(i)-1);
        obj = obj.withEqs(obj.grasped_geometry.min_eig_A*[obj.XCTellipsoid{i}(4,1);obj.XCTellipsoid{i}(4,2);obj.XCTellipsoid{i}(4,3)]-obj.grasped_geometry.A_ellipsoid'*obj.c(:,i));
        
        % xc'*A_ellipsoid'*c = min_eig_A*t_ellipsoid
        obj = obj.withEqs(replaceBilinearProduct(obj.xc(:,i)'*obj.grasped_geometry.A_ellipsoid'*obj.c(:,i),[obj.xc(:,i);obj.c(:,i)],obj.XCC{i}(1:6,1:6))-obj.grasped_geometry.min_eig_A*obj.t_ellipsoid(i));
        % Also add the constraint that the contact point is on the
        % separating hyperplane c'x+d = 0, and the ellipsoid samples are in the
        % halfspace c'x+d<=0
        obj = obj.withPos(-(obj.c(:,i)'*ellipsoid_sample+obj.d(i)*ones(1,size(ellipsoid_sample,2))));
      end
    end
    
    function obj = setGraspedCylinder(obj)
      for i = 1:obj.num_contacts
        % add the constraint that xc(1)^2+xc(2)^2 = 1
        obj = obj.withEqs(obj.XCC{i}(1,1)+obj.XCC{i}(2,2)-1);
        % add the constraint that -1<=xc(3)<=1
        obj = obj.withPos(1-obj.xc(3,i));
        obj = obj.withPos(obj.xc(3,i)+1);
        % add the constraint that c(:,i) = obj.R_cylinder*[xc(1:2,i);0];
        obj = obj.withEqs(obj.c(:,i)-obj.grasped_geometry.R_cylinder*[obj.xc(1:2,i);0]);
        % add the constraint that c(:,i)*c(:,i)' =
        % obj.R_cylinder*[xc(1:2,i);0]*[xc(1:2,i);0]'*obj.R_cylinder'
        obj = obj.withEqs(obj.XCC{i}(4:6,4:6)-obj.grasped_geometry.R_cylinder*[obj.XCC{i}(1:2,1:2) zeros(2,1);zeros(1,3)]*obj.grasped_geometry.R_cylinder');
        % xc(:,i)*c(:,i)' = xc(:,i)*[xc(1:2,i);0]'*obj.R_cylinder'
        obj = obj.withEqs(obj.XCC{i}(1:3,4:6)-[obj.XCC{i}(1:3,1:2) zeros(3,1)]*obj.grasped_geometry.R_cylinder');
      end 
    end
  end
  
  methods(Abstract,Access = protected)
    obj = addXCC(obj);
  end
end
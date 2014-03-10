classdef RigidBodyAddedMass < RigidBodyForceElement
    
    properties
        kinframe;  %index of parent node
        MaOrigin; %added mass of body, as seen at xyz origin
        MaBody; %added mass, as seen by the parent body coordinates
        I; %mass matrix of body about joint
        inertia; %inertia of parent body about com
        com; %com of parent body
        mass; %mass of parent body
    end
    
    methods
        function obj = RigidBodyAddedMass(frame_id, MaOrigin, MaBody)
            % creates a force element that adds buoyancy at new frame location
            % @param frame_id = RigidBodyFrame specifying the location added mass
            % coefficients have been located
            % @param Ma = added mass matrix (in Featherstone's spatial form
            % for consistency, not 123456 as is literature norm)
            
            typecheck(frame_id,'numeric');
            obj.kinframe = frame_id;
            obj.MaOrigin = MaOrigin;
            obj.MaBody = MaBody;
            
        end
        function obj = pullParentInertia(obj,manip)
            % Pulls original mass properties of the parent body
            frame = getFrame(manip,obj.kinframe);
            
            obj.mass = manip.body(frame.body_ind).mass;
            obj.com = manip.body(frame.body_ind).com;
            obj.inertia = manip.body(frame.body_ind).inertia;
            obj.I = manip.body(frame.body_ind).I;
        end
        function manip = augmentParentInertia(obj,manip)
            % augments the inertia of the parent link to include added mass
            % This is the sketchy part, perhaps a better way to implement?
            frame = getFrame(manip,obj.kinframe);
            body_ind = frame.body_ind;
            
            %Find the spatial inertia matrix
            Imass = mcI(obj.mass,obj.com,obj.inertia);
            
            %Augment spatial inertia with added mass
            manip = manip.setBody(body_ind,manip.body(body_ind).setOnlyInertial(Imass+obj.MaBody));
        end
        function force = computeSpatialForce(obj,manip,q,qd)
            % Adds the gravity correction from added mass
            % This force is ONLY necessary because the Featherstone solver
            % imposes gravitational acceleration at the base of the
            % manipulator rather than imposing a gravity for at each node
            % Imposing at the base assumes that F_gravity = (M+Ma)*g,
            % rather than F_gravity = M*g
            % Essentially, gravitional and inertial mass are NOT the 
            % same if added mass forces are assumed (isn't physics fun?)
            
            frame = getFrame(manip,obj.kinframe);
            body_ind = frame.body_ind;
            kinsol = doKinematics(manip,q);
            
            %Determine gravitation acceleration in local body frame
            gravAccLocal = bodyKin(manip,kinsol,body_ind,[manip.gravity,zeros(3,1)]); 
            gravAccLocal = gravAccLocal(:,1)-gravAccLocal(:,2);
            gravAccLocal = [zeros(3,1);gravAccLocal]; %Augment so that is force is [torques;forces]
            
            %Correction force (-Ma*g) to mitigate the acceleration of the base
            forceLocal = -obj.MaBody*gravAccLocal;

            %Transform from body to joint coordinates
            force = sparse(6,getNumBodies(manip));
            force(:,body_ind) = manip.body(body_ind).X_joint_to_body'*forceLocal;
        end
    end
    
    methods (Static)
        function [model,obj] = parseURDFNode(model,robotnum,node,options)
            name = char(node.getAttribute('name'));
            name = regexprep(name, '\.', '_', 'preservecase');
            
            elNode = node.getElementsByTagName('parent').item(0);
            parent = findLinkInd(model,char(elNode.getAttribute('link')),robotnum);
            
            xyz=zeros(3,1); rpy=zeros(3,1);
            elnode = node.getElementsByTagName('origin').item(0);
            if ~isempty(elnode)
                if elnode.hasAttribute('xyz')
                    xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
                end
                if elnode.hasAttribute('rpy')
                    rpy = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('rpy'))),3,1);
                    if any(rpy)
                        warning('Added mass about a rotated frame has not yet been implemented, reverting to rpy=[0;0;0]');
                        rpy = zeros(3,1);
                    end
                end
            end
            [model,frame_id] = addFrame(model,RigidBodyFrame(parent,xyz,rpy,[name,'_frame']));
            
            %Import added mass matrix (using usual m11, m22, etc form from literature)
            MaOrigin = zeros(6,6);
            for i=1:6
                for j = i:6
                    mij_string = ['m' mat2str(i,1) mat2str(j,1)];
                    if node.hasAttribute(mij_string)
                        MaOrigin(i,j) = parseParamString(model,robotnum,char(node.getAttribute(mij_string)));
                        MaOrigin(j,i) = MaOrigin(i,j);
                    end
                end
            end
            
            % Reverse coordinates so that in Featherstone's spatial form
            MaOrigin = MaOrigin([4:6 1:3],[4:6 1:3]);
            
            %Include xyz offset into added mass matrix
            MaBody = moveSpatialMassMatrix(xyz,MaOrigin);
            
            %Declare object
            obj = RigidBodyAddedMass(frame_id, MaOrigin, MaBody);
            obj = pullParentInertia(obj,model);
            obj.name = name;
            
            % Sets the inertia of the parent node to include the added mass
            model = augmentParentInertia(obj,model);
            
        end
        
        
    end
    
end

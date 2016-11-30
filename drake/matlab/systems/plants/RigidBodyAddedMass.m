classdef RigidBodyAddedMass < RigidBodyForceElement
    
    properties
        kinframe  %index of frame location
        MaOrigin = zeros(6,6); %added mass of body, as seen at xyz origin
        MaBody = zeros(6,6); %added mass, as seen by the parent body coordinates
    end
    
    methods
        function obj = RigidBodyAddedMass(frame_id, manip, MaOrigin)
            % creates a force element that adds added-mass at new frame location
            % @param frame_id = RigidBodyFrame index specifying the location added mass
            % coefficients have been located
            % @param manip = RigidBodyManipulator that the frame_id
            % references
            % @param MaOrigin = added mass matrix as seen by origin of 
            % added mass frame (in Featherstone's spatial form 
            % for consistency, not 123456 as is literature norm)  
            
            typecheck(frame_id,'numeric');
            obj.kinframe = frame_id;
            obj.MaOrigin = MaOrigin;
            
            frame = getFrame(manip,obj.kinframe);
            xyz = frame.T(1:3,4);
            
            %Include xyz offset into added mass matrix
            obj.MaBody = RigidBodyAddedMass.moveSpatialMassMatrix(xyz,obj.MaOrigin);
            
        end
        function manip = augmentParentInertia(obj,manip)
            %Augments the inertia of the parent link to include added mass
            frame = getFrame(manip,obj.kinframe);
            body_ind = frame.body_ind;
            
            %Find the spatial inertia matrix, only including mass
            Imass = manip.body(frame.body_ind).Imass;
            
            %Augment spatial inertia with added mass
            manip = manip.setBody(body_ind,manip.body(body_ind).setInertial(Imass,obj.MaBody));
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
            gravAccLocal = [zeros(3,1);gravAccLocal]; %Augment so that the force is [torques;forces]
            
            %Correction force (-Ma*g) to mitigate the acceleration of the base
            forceLocal = -obj.MaBody*gravAccLocal;

            %Transform from body to joint coordinates
            force = sparse(6,getNumBodies(manip));
            force(:,body_ind) = forceLocal;
        end
    end
    
    methods (Static)
        function [model,obj] = parseURDFNode(model,name,robotnum,node,options)
            elNode = node.getElementsByTagName('parent').item(0);
            parent = findLinkId(model,char(elNode.getAttribute('link')),robotnum);
            
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
                       
            %Declare object
            obj = RigidBodyAddedMass(frame_id, model, MaOrigin);
            obj.name = name;
            
            % Sets the inertia of the parent node to include the added mass
            model = augmentParentInertia(obj,model);
            
        end
        function I_new = moveSpatialMassMatrix(c,I_old)
            % Moves an added mass matrix (or spatial mass matrix) by c.
            % Equivalent to mcI(m,c,Icm) but works for non-scalar m and
            % allows for inertias to not be about center of mass
            % @param I_old is spatial mass matrix in [angular;velo] form as
            % consistent with Featherstone's notation
            % @param c [3x1] is the vector from the NEW origin to the OLD origin
            % algebra tested with test/moveSpatialMassMatrixTest
            
            M11 = I_old(1:3,1:3); M12 = I_old(1:3,4:6);
            M21 = I_old(4:6,1:3); M22 = I_old(4:6,4:6);
            C = [  0,    -c(3),  c(2);
                c(3),  0,    -c(1);
                -c(2),  c(1),  0 ];
            
            % Version in usual [velo,angular] coordinates
            %I_new = [M11 -M11*C+M12; C*M11+M21 M22-C*M11*C-M21*C+C*M12];
            
            % Version in Featherstone's notation: swaps M11 for M22 and M12 for M21,
            % and order of elements
            I_new = [M11-C*M22*C-M12*C+C*M21 C*M22+M12; -M22*C+M21 M22];
        end
    end
    
end

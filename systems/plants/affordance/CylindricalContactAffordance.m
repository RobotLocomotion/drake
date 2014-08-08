classdef CylindricalContactAffordance < ContactAffordance
    properties
        % when is_fixed = false, all quantities are in the local frame
        % otherwise, all quantities are in the world frame
        poles
        radius
        height
        center
    end
    methods
        function obj = CylindricalContactAffordance(robot,body_ind,name,poles,radius,is_fixed,mu)
            sizecheck(poles,[3,2]);
            sizecheck(radius,[1,1]);
            if(nargin<7)
                mu = 1;
            end
            obj = obj@ContactAffordance(robot,body_ind,name,is_fixed,mu);
            obj.poles = poles;
            obj.radius = radius;
            obj.height = norm(obj.poles(:,2)-obj.poles(:,1));
            obj.center = (obj.poles(:,1)+obj.poles(:,2))/2;
        end
        
        function [dist,dDist] = distancePts2Aff(obj,q,pts)
            n_pts = size(pts,2);
            nq = obj.robot.getNumPositions();
            sizecheck(q,[nq,1]);
            if(obj.is_fixed)
                poles_abs = obj.poles;
                dpoles_absdq = zeros(6,nq);
            else
                kinsol = doKinematics(obj.robot,q);
                [poles_abs,dpoles_absdq] = forwardKin(obj.robot,kinsol,obj.body_ind,obj.poles,0);
            end
            axis = poles_abs(:,2)-poles_abs(:,1);
            axis_norm = norm(axis);
            daxis_unitdq = dpoles_absdq(4:6,:)-dpoles_absdq(1:3,:);
            hypotenuse = pts-bsxfun(@times,poles_abs(:,1),ones(1,n_pts));
            hypo_norm = sqrt(sum(hypotenuse.*hypotenuse,1));
            % let a = hypotenuse, b = axis_unit, then the distance is
            % |a-a'*b/(b'*b)*b|
            dist = sqrt((hypo_norm').^2*axis_norm^2-(hypotenuse'*axis).^2)./axis_norm;
            dDistdhypo = (hypotenuse'-hypotenuse'*axis*axis'/axis_norm^2)./bsxfun(@times,dist,ones(1,3)); % This is arranged as [dDist(1)/dhypo(:,1);dDist(2)/dhypo(:,2);...]
            dDistdaxis =  (-(bsxfun(@times,hypotenuse'*axis,ones(1,3)).*hypotenuse'-(hypotenuse'*axis).^2*axis')/axis_norm^2)./bsxfun(@times,dist,ones(1,3));
            dDistdq = dDistdhypo*(-dpoles_absdq(1:3,:))+dDistdaxis*daxis_unitdq;
            dDistdpts = sparse(reshape(bsxfun(@times,(1:n_pts),ones(3,1)),[],1),...
                (1:3*n_pts)',reshape(dDistdhypo',[],1));
            dist = dist-obj.radius;
            dDist = [dDistdq dDistdpts];
        end
        
        function [res,dRes] = additionalResPts2Aff(obj,q,pts)
            % This function determines if the projection of pts to the axis
            % is within the cylinder. The projection is in the cylinder if
            % -obj.height/2<=res<=obj.height/2
            n_pts = size(pts,2);
            nq = obj.robot.getNumPositions();
            sizecheck(q,[nq,1]);
            if(obj.is_fixed)
                center_abs = obj.center;
                poles_abs = obj.poles;
                dpoles_absdq = zeros(6,nq);
                dcenter_absdq = zeros(3,nq);
            else
                kinsol = doKinematics(obj.robot,q);
                [poles_abs,dpoles_absdq] = forwardKin(obj.robot,kinsol,obj.body_ind,obj.poles,0);
                center_abs = (poles_abs(:,1)+poles_abs(:,2))/2;
                dcenter_absdq = (dpoles_absdq(1:3,:)+dpoles_absdq(4:6,:))/2;
            end
            axis = poles_abs(:,2)-poles_abs(:,1);
            daxisdq = dpoles_absdq(4:6,:)-dpoles_absdq(1:3,:);
            axis_norm = norm(axis);
            hypotenuse = pts-bsxfun(@times,center_abs,ones(1,n_pts));
            res = hypotenuse'*axis/axis_norm;
            dresdhypo = bsxfun(@times,axis'/axis_norm,ones(n_pts,1));
            dresdaxis = (hypotenuse'*axis_norm-hypotenuse'*axis*axis'/axis_norm)/axis_norm^2;
            dresdq = dresdhypo*(-dcenter_absdq)+dresdaxis*daxisdq;
            dresdpts = sparse(reshape(bsxfun(@times,(1:n_pts),ones(3,1)),[],1),...
                (1:3*n_pts)',reshape(dresdhypo',[],1));
            dRes = [dresdq dresdpts];
        end
    end
end
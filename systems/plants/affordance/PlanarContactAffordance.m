classdef PlanarContactAffordance < ContactAffordance
    properties
        % normal and pt are in the world coordinate if the plane is fixed,
        % otherwise it is in the plane coordinate.
        normal % The normal vector
        pt % A random point on the plane
    end
    
    methods
        function obj = PlanarContactAffordance(robot,body_ind,name,pt,normal,is_fixed,mu)
            sizecheck(normal,[3,1]);
            sizecheck(pt,[3,1]);
            if(nargin<7)
                mu = 1;
            end
            obj = obj@ContactAffordance(robot,body_ind,name,is_fixed,mu);
            obj.normal = normal;
            obj.pt = pt;
        end
        
        function [dist,dDist] = distancePts2Aff(obj,q,pts)
            sizecheck(pts,[3,nan]);
            n_pts = size(pts,2);
            nq = obj.robot.getNumPositions();
            sizecheck(q,[nq,1]);
            if(obj.is_fixed)
                normal_abs = obj.normal;
                pt_abs = obj.pt;
                dnormal_absdq = zeros(3,nq);
                dpt_absdq = zeros(3,nq);
            else
                kinsol = doKinematics(obj.robot,q);
                [pts_abs,dpts_absdq] = forwardKin(obj.robot,kinsol,obj.body_ind,[obj.normal,obj.pt,zeros(3,1)],0);
                normal_abs = pts_abs(:,1)-pts_abs(:,3);
                dnormal_absdq = dpts_absdq(1:3,:)-dpts_absdq(7:9,:);
                pt_abs = pts_abs(2,:);
                dpt_absdq = dpts_absdq(4:6,:);
            end
            dist = (pts-bsxfun(@times,pt_abs,ones(1,n_pts)))'*normal_abs;
            if(nargout>1)
                dDistdq = repmat(normal_abs'*(-dpt_absdq),n_pts,1)+(pts-bsxfun(@times,pt_abs,ones(1,n_pts)))'*dnormal_absdq;
                dDistdpts = sparse(reshape(bsxfun(@times,[1;1;1],(1:n_pts)),[],1),...
                    (1:n_pts*3)',reshape(bsxfun(@times,normal_abs,ones(1,n_pts)),[],1));
                dDist = [dDistdq dDistdpts];
            end
        end
    end
end
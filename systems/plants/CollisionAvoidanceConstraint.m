classdef CollisionAvoidanceConstraint < ActionKinematicConstraint
    properties
    end
    methods
        function obj = CollisionAvoidanceConstraint(robot,body_ind,tspan,name,obstacles)
            body_pts = [0;0;0];
            worldpos = struct('max',inf(3,1),'min',-inf(3,1));
            contact_state0 = {ActionKinematicConstraint.UNDEFINED_CONTACT};
            contact_statei = {ActionKinematicConstraint.COLLISION_AVOIDANCE};
            contact_statef = {ActionKinematicConstraint.UNDEFINED_CONTACT};
            contact_distance = {struct('max',inf,'min',0)};
            if(~iscell(obstacles)&&isa(obstacles,'RigidBody'))
                obstacles = {obstacles};
            elseif(iscell(obstacles))
                for i = 1:length(obstacles)
                    if(~isa(obstacles{i},'RigidBody'))
                        error('each entry of obstacles must be a RigidBody object');
                    end
                end
            else
                error('obstacle input is not correct');
            end
            obj = obj@ActionKinematicConstraint(robot,body_ind,body_pts,worldpos,tspan,name,contact_state0,contact_statei,contact_statef,obstacles,contact_distance);
        end
    end
end
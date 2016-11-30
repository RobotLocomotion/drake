function ret = isBodyPartOfRobot(model, body, robotnum)
% determines whether a given body is part of a robot specified by robotnum.
% @param body a RigidBody
% @param robotnum vector of integers corresponding to the
% RigidBody.robotnum field.
% @retval ret whether or not the body is part of a robot.

if any(robotnum < 0)
  ret = true;
else
  ret = any(body.robotnum == robotnum);
end

end

function a = uniformlyRandomAxisAngle()
axis = randn(3, 1);
axis = axis / norm(axis);
angle = (rand - 0.5) * 2 * pi;
a = [axis; angle];
end


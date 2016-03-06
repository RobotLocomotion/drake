function g = datan2(x,y)
% computate the gradient of atan2(y,x) w.r.t x and y
% @retval g = [datan2(y,x)/dx datan2(y,x)/dy]
g = [-y x]/(x^2+y^2);
end
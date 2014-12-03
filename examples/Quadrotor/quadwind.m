function wind = quadwind(quadpos)
% quadpos is [xquad;yquad;zquad]

quadpos = quadpos;
xquad = quadpos(1);
yquad = quadpos(2);
zquad = quadpos(3);

xwind = 0;

%ywind = zquad^2;
ywind = zquad;
%ywind = 5;

zwind = 0;

wind = [xwind;ywind;zwind];
end
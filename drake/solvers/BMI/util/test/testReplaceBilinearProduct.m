function testReplaceBilinearProduct
checkDependency('spotless');
prog = spotsosprog();
[prog,y] = prog.newFree(3,1);
[prog,x] = prog.newFree(3,1);
[prog,X] = prog.newSym(3);
Q = reshape(1:9,3,3);
expr1 = x'*Q*x;
expr1X = replaceBilinearProduct(expr1,x,X);
if(~isequal(sum(sum(Q.*X)),expr1X))
  error('fails to replace quadratic forms');
end
a = (10:12)';
b = 13;
expr2 = x'*Q*x+a'*x+b;
expr2X = replaceBilinearProduct(expr2,x,X);
if(~isequal(sum(sum(Q.*X))+a'*x+b,expr2X))
  error('fails to replace a second order polynomial');
end
c = [2; 3; 4];
expr3 = x'*Q*(x+y)+y'*Q*y+c'*y+a'*x+b+x'*Q*x*(y'*Q*y);
expr3X = replaceBilinearProduct(expr3,x,X);
if(~isequal(sum(sum(Q.*X))+x'*Q*y+y'*Q*y+c'*y+a'*x+b+sum(sum(Q.*X))*(y'*Q*y),expr3X))
  error('fails to replace x when another msspoly is also involved');
end
[prog,z] = prog.newFree(3,1);
expr4 = expr3+x'*z+y'*z+z'*z+c'*z;
expr4X = replaceBilinearProduct(expr4,x,X);
if(~isequal(expr4X,expr3X+x'*z+y'*z+z'*z+c'*z))
    error('fails to replace x when another msspoly is also involved');
end
expr = replaceBilinearProduct([expr1 expr3;expr2 expr4],x,X);
if(~isequal(expr(1,1),expr1X) || ~isequal(expr(2,1),expr2X) || ~isequal(expr(1,2),expr3X) || ~isequal(expr(2,2),expr4X))
  error('fails to replace a matrix expression');
end
end
function testBMI()
 % simple problem x^2+y^2=1,x+y+z >1, xz = 2
 checkDependency('spotless');
 p = BMIspotless();
 [p,x] = p.newFree(1,1);
 [p,y] = p.newFree(1,1);
 [p,z] = p.newFree(1,1);
 [p,XY] = p.newSym(2);
 [p,XZ] = p.newSym(2);
 p = p.withEqs(XY(1,1)+XY(2,2)-1);
 p = p.withPos(x+y+z-1);
 p = p.withEqs(XZ(1,2)-2);
 p = p.withEqs(XZ(1,1)-XY(1,1));
 p = p.addBilinearVariable([x;y],XY);
 p = p.addBilinearVariable([x;z],XZ);
 
 [solver_sol,info] = p.optimize();
 if(info~= 1)
   error('BMIspotless should solve this problem')
 end
  checkSolution(solver_sol,x,y,z);
  % add backoff
  p.final_backoff_flag = true;
  [solver_sol,info,itr,time] = p.optimize();
  if(info~=1)
    error('BMIspotless should solve this problem');
  end
  checkSolution(solver_sol,x,y,z);
  
 % Test using the previous solution.
 [solver_sol,info,itr] = p.optimize(double(solver_sol.eval(p.w)));
 if(itr>1)
   error('BMIspotless should converge in the first iteration');
 end
 checkSolution(solver_sol,x,y,z);
end

function checkSolution(solver_sol,x,y,z)
 x_sol = double(solver_sol.eval(x));
 y_sol = double(solver_sol.eval(y));
 z_sol = double(solver_sol.eval(z));
 if(abs(x_sol^2+y_sol^2-1)>1e-3)
   error('not satisfying x^2+y^2=1');
 end
 if(x_sol+y_sol+z_sol-1<-1e-4)
   error('not satisfying x+y+z>1');
 end
 if(abs(x_sol*z_sol-2)>1e-4)
   error('not satisfying xz=2');
 end
end
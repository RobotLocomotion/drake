function testPolyhedronNoCollision()
checkDependency('spotless');
verts1 = randn(3,100);
verts2 = randn(3,100);
p = PolyhedronNoCollision(verts1,verts2);
solver_sol = p.optimize();
sol = p.retrieveSolution(solver_sol);
p.plotSolution(sol);
end

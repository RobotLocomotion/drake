import sys
import os

# sys.path.append('/afs/csail.mit.edu/u/r/rhjiang/Documents/code/drake-build/install/lib/python3.10/site-packages')
# os.environ["GRB_LICENSE_FILE"] = "/afs/csail.mit.edu/u/r/rhjiang/gurobi.lic"
# os.environ["MOSEKLM_LICENSE_FILE"] = "/afs/csail.mit.edu/u/r/rhjiang/mosek/mosek.lic"

import numpy as np
import pydrake.solvers as solvers_drake
from pydrake.solvers import MathematicalProgram
from pydrake.geometry.optimization import HPolyhedron
import pydrake.math as pm
from gurobipy import *
import scipy.sparse as sp

gurobi_solver = solvers_drake.GurobiSolver()
gurobi_license = gurobi_solver.AcquireLicense()
mosek_solver = solvers_drake.MosekSolver()
clp_solver = solvers_drake.ClpSolver()
mosek_license = mosek_solver.AcquireLicense()


def is_empty(polytope,solver=clp_solver):
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(np.shape(polytope.A())[1],'x')
    prog.AddLinearConstraint(polytope.A(), -np.inf*np.ones(len(polytope.b())),polytope.b(),x)
    prog.AddLinearCost(np.ones(polytope.ambient_dimension()),0,x)
    result = solver.Solve(prog)
    return not result.is_success()

def check_intersection(polytope1, polytope2,solver=clp_solver):
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(polytope1.ambient_dimension(),'x')
    polytope1.AddPointInSetConstraints(prog,x)
    polytope2.AddPointInSetConstraints(prog,x)
    prog.AddLinearCost(np.ones(polytope1.ambient_dimension()),0,x)
    result = solver.Solve(prog)
    return result.is_success()

def one_poly_inter_active(polytope1,polytope2):
    # get the hyperplanes belonging to polytope1 that are not redundant in the intersection 
    # between polytope1 and polytope2.
    inter = polytope1.Intersection(polytope2)
    redundant = list(inter.FindRedundant())
    redundant_polytope_1 = [ind for ind in list(redundant) if ind < len(polytope1.b())]
    mask = np.ones(len(polytope1.b()),dtype=bool)
    mask[redundant_polytope_1] = False
    return polytope1.A()[mask], polytope1.b()[mask]

def find_max_removable_faces(inbody,circumbody,M=10000,solver = gurobi_solver,use_initial_guess=True):
    # maximum number of faces that can be removed from inbody subject to remaining subset of circumbody
    Hx = inbody.A()
    hx = inbody.b()
    Hy = circumbody.A()
    hy = circumbody.b()
    # TODO normalize matrices

    Nx = np.shape(Hx)[0]
    Ny = np.shape(Hy)[0]

    if np.isscalar(M):
        M = M * np.ones((Ny,Nx)) # repeat value in matrix

    # print("here1")
    prog = MathematicalProgram()
    Lambda = prog.NewContinuousVariables(Ny,Nx,"Lambda")
    # print("here2")
    prog.AddBoundingBoxConstraint(0,np.inf,Lambda) # TODO move upper bound to M?  or no?
    # prog.AddLinearEqualityConstraint((pm.matmul(Lambda,Hx)).flatten(),Hy.flatten())
    for i in range(Ny):
        prog.AddLinearEqualityConstraint(pm.matmul(Lambda[np.newaxis,i],Hx)[0],Hy[i])
    # print("here3")
    prog.AddLinearConstraint(pm.matmul(Lambda,hx),-np.inf*np.ones(Ny),hy)
    # print("here4")

    b = prog.NewBinaryVariables(Nx,"b")
    for j in range(Nx):
        # if j%10 == 0:
        #     print(j)
        for i in range(Ny):
            prog.AddLinearConstraint(Lambda[i,j] - M[i,j]*b[j],-np.inf,0)
        

    prog.AddLinearCost(np.sum(b))

    if use_initial_guess:
        prog.SetInitialGuess(b,np.ones(Nx))
        if np.allclose(Hx,Hy):
            # print("here4.5")
            prog.SetInitialGuess(Lambda,np.eye(Nx))

    result = solver.Solve(prog)
    # print("here5")
    return result.GetSolution(b), result.GetSolution(Lambda), result

def find_max_removable_faces_gurobipy(inbody,circumbody,use_big_M = False,M=100,use_initial_guess=True):
    # maximum number of faces that can be removed from inbody subject to remaining subset of circumbody
    Hx_full = inbody.A()
    Hx = sp.csr_matrix(Hx_full)
    hx = inbody.b()
    Hy_full = circumbody.A()
    Hy = sp.csr_matrix(Hy_full)
    hy = circumbody.b()
    # TODO normalize matrices

    Nx = np.shape(Hx)[0]
    Ny = np.shape(Hy)[0]

    if np.isscalar(M) == 1:
        M = M * np.ones((Ny,Nx)) # repeat value in matrix

    # Env(empty=True).setParam('LogToConsole', 0)
    print("here1")
    m = Model()
    m.Params.LogToConsole = 0
    b = m.addMVar(Nx,vtype='B')
    Lambda = m.addMVar((Ny,Nx),lb = 0,vtype = 'C')
    print("here2")
    m.addConstr(Lambda @ Hx == Hy)
    # TODO might be faster to do row-wise in a loop, as below.  OR, maybe make Hx and Hy sparse?
    # for i in range(Ny):
    #     m.addConstr(Lambda[i]@Hx == Hy[i])
    print("here3")
    m.addConstr(Lambda @ hx <= hy)
    print("here4")

    if use_big_M:
        print("using big-M formulation with Gurobi")
        for j in range(Nx):
            for i in range(Ny):
                m.addConstr(Lambda[i,j] - M[i,j]*b[j]<=0)
    else:
        print("using indicator constraint formulation with Gurobi")
        for i in range(Nx):
            for j in range(Ny):
                m.addConstr((b[i]==0) >> (Lambda[j,i]==0))
        
    m.setObjective(np.ones(Nx) @ b, GRB.MINIMIZE)

    if use_initial_guess:
        b.Start = np.ones(Nx)
        if np.allclose(Hx_full,Hy_full):
            print("here4.5")
            Lambda.Start = np.eye(Nx)

    m.optimize()
    print("here5")
    return b.X, Lambda.X

def get_reduced_polytope(polytope,b):
    mask = b>0.999
    return HPolyhedron(polytope.A()[mask],polytope.b()[mask])

def check_if_subset(X,Y):
    # check if HPolyhedron X is a subset of Y
    if X.IsEmpty():
        return True
    elif Y.IsEmpty():
        return False
    
    Hx = X.A()
    hx = X.b()
    Hy = Y.A()
    hy = Y.b()
    Nx = np.shape(Hx)[0]
    Ny = np.shape(Hy)[0]
    prog = MathematicalProgram()
    Lambda = prog.NewContinuousVariables(Ny,Nx,"Lambda")
    prog.AddBoundingBoxConstraint(0,np.inf,Lambda)
    prog.AddLinearEqualityConstraint((np.dot(Lambda,Hx)).flatten(),Hy.flatten())
    prog.AddLinearConstraint(np.dot(Lambda,hx),-np.inf*np.ones(Ny),hy)
    result = gurobi_solver.Solve(prog)
    
    return result.is_success()

def print_inequalities(H,h):
    H = np.where(np.abs(H) <= 1e-13, 0, H)
    h = h.flatten()
    h = np.where(np.abs(h) <= 1e-13, 0, h)
    for row in range(len(h)):
        print(f'{H[row,0]}x + {H[row,1]}y<{h[row]}')
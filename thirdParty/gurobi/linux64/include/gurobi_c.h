/* Copyright 2013, Gurobi Optimization, Inc. */
 
#ifndef _GUROBI_C_H
#define _GUROBI_C_H
 
#include <stdio.h>

typedef struct _GRBmodel GRBmodel;
typedef struct _GRBenv GRBenv;

#if defined(_WIN64) && !defined(WIN64)
#define WIN64
#endif

#if defined(_WIN32) && !defined(_WIN64) && !defined(WIN32)
#define WIN32
#endif

#if !defined(WIN32) && !defined(WIN64)
#define __cdecl
#define __stdcall
#endif

/* Version numbers */

#define GRB_VERSION_MAJOR     5
#define GRB_VERSION_MINOR     1
#define GRB_VERSION_TECHNICAL 0

/* Error codes */

#define GRB_ERROR_OUT_OF_MEMORY            10001
#define GRB_ERROR_NULL_ARGUMENT            10002
#define GRB_ERROR_INVALID_ARGUMENT         10003
#define GRB_ERROR_UNKNOWN_ATTRIBUTE        10004
#define GRB_ERROR_DATA_NOT_AVAILABLE       10005
#define GRB_ERROR_INDEX_OUT_OF_RANGE       10006
#define GRB_ERROR_UNKNOWN_PARAMETER        10007
#define GRB_ERROR_VALUE_OUT_OF_RANGE       10008
#define GRB_ERROR_NO_LICENSE               10009
#define GRB_ERROR_SIZE_LIMIT_EXCEEDED      10010
#define GRB_ERROR_CALLBACK                 10011
#define GRB_ERROR_FILE_READ                10012
#define GRB_ERROR_FILE_WRITE               10013
#define GRB_ERROR_NUMERIC                  10014
#define GRB_ERROR_IIS_NOT_INFEASIBLE       10015
#define GRB_ERROR_NOT_FOR_MIP              10016
#define GRB_ERROR_OPTIMIZATION_IN_PROGRESS 10017
#define GRB_ERROR_DUPLICATES               10018
#define GRB_ERROR_NODEFILE                 10019
#define GRB_ERROR_Q_NOT_PSD                10020
#define GRB_ERROR_QCP_EQUALITY_CONSTRAINT  10021

/* Constraint senses */

#define GRB_LESS_EQUAL    '<'
#define GRB_GREATER_EQUAL '>'
#define GRB_EQUAL         '='

/* Variable types */

#define GRB_CONTINUOUS 'C'
#define GRB_BINARY     'B'
#define GRB_INTEGER    'I'
#define GRB_SEMICONT   'S'
#define GRB_SEMIINT    'N'

/* Objective sense */

#define GRB_MINIMIZE 1
#define GRB_MAXIMIZE -1

/* SOS types */

#define GRB_SOS_TYPE1 1
#define GRB_SOS_TYPE2 2

/* Numeric constants */

#define GRB_INFINITY  1e100
#define GRB_UNDEFINED 1e101

/* Limits */

#define GRB_MAX_NAMELEN    255
#define GRB_MAX_STRLEN     512
#define GRB_MAX_CONCURRENT 16
/* Copyright 2011, Gurobi Optimization, Inc. */

/* Query interface */

int __stdcall
  GRBgetattrinfo(GRBmodel *model, const char *attrname, int *datatypeP,
                 int *sizeP, int *settableP);
int __stdcall
  GRBisattravailable(GRBmodel *model, const char *attrname);
int __stdcall
  GRBgetintattr(GRBmodel *model, const char *attrname, int *valueP);
int __stdcall
  GRBsetintattr(GRBmodel *model, const char *attrname, int newvalue);
int __stdcall
  GRBgetintattrelement(GRBmodel *model, const char *attrname,
                       int element, int *valueP);
int __stdcall
  GRBsetintattrelement(GRBmodel *model, const char *attrname,
                       int element, int newvalue);
int __stdcall
  GRBgetintattrarray(GRBmodel *model, const char *attrname,
                     int first, int len, int *values);
int __stdcall
  GRBsetintattrarray(GRBmodel *model, const char *attrname,
                     int first, int len, int *newvalues);
int __stdcall
  GRBgetintattrlist(GRBmodel *model, const char *attrname,
                    int len, int *ind, int *values);
int __stdcall
  GRBsetintattrlist(GRBmodel *model, const char *attrname,
                    int len, int *ind, int *newvalues);
int __stdcall
  GRBgetcharattrelement(GRBmodel *model, const char *attrname,
                        int element, char *valueP);
int __stdcall
  GRBsetcharattrelement(GRBmodel *model, const char *attrname,
                        int element, char newvalue);
int __stdcall
  GRBgetcharattrarray(GRBmodel *model, const char *attrname,
                      int first, int len, char *values);
int __stdcall
  GRBsetcharattrarray(GRBmodel *model, const char *attrname,
                      int first, int len, char *newvalues);
int __stdcall
  GRBgetcharattrlist(GRBmodel *model, const char *attrname,
                     int len, int *ind, char *values);
int __stdcall
  GRBsetcharattrlist(GRBmodel *model, const char *attrname,
                     int len, int *ind, char *newvalues);
int __stdcall
  GRBgetdblattr(GRBmodel *model, const char *attrname, double *valueP);
int __stdcall
  GRBsetdblattr(GRBmodel *model, const char *attrname, double newvalue);
int __stdcall
  GRBgetdblattrelement(GRBmodel *model, const char *attrname,
                       int element, double *valueP);
int __stdcall
  GRBsetdblattrelement(GRBmodel *model, const char *attrname,
                       int element, double newvalue);
int __stdcall
  GRBgetdblattrarray(GRBmodel *model, const char *attrname,
                     int first, int len, double *values);
int __stdcall
  GRBsetdblattrarray(GRBmodel *model, const char *attrname,
                     int first, int len, double *newvalues);
int __stdcall
  GRBgetdblattrlist(GRBmodel *model, const char *attrname,
                    int len, int *ind, double *values);
int __stdcall
  GRBsetdblattrlist(GRBmodel *model, const char *attrname,
                    int len, int *ind, double *newvalues);
int __stdcall
  GRBgetstrattr(GRBmodel *model, const char *attrname, char **valueP);
int __stdcall
  GRBsetstrattr(GRBmodel *model, const char *attrname, const char *newvalue);
int __stdcall
  GRBgetstrattrelement(GRBmodel *model, const char *attrname,
                       int element, char **valueP);
int __stdcall
  GRBsetstrattrelement(GRBmodel *model, const char *attrname,
                       int element, const char *newvalue);
int __stdcall
  GRBgetstrattrarray(GRBmodel *model, const char *attrname,
                     int first, int len, char **values);
int __stdcall
  GRBsetstrattrarray(GRBmodel *model, const char *attrname,
                     int first, int len, char **newvalues);
int __stdcall
  GRBgetstrattrlist(GRBmodel *model, const char *attrname,
                    int len, int *ind, char **values);
int __stdcall
  GRBsetstrattrlist(GRBmodel *model, const char *attrname,
                    int len, int *ind, char **newvalues);

/* Callback */
#define CB_ARGS GRBmodel *model, void *cbdata, int where, void *usrdata

int __stdcall
  GRBsetcallbackfunc(GRBmodel *model,
                     int (__stdcall *cb)(CB_ARGS),
                     void  *usrdata);
int __stdcall
  GRBgetcallbackfunc(GRBmodel *model,
                     int (__stdcall **cbP)(CB_ARGS));
int __stdcall
  GRBcbget(void *cbdata, int where, int what, void *resultP);
int __stdcall
  GRBcbsolution(void *cbdata, const double *solution);
int __stdcall
  GRBcbcut(void *cbdata, int cutlen, const int *cutind, const double *cutval,
          char cutsense, double cutrhs);
int __stdcall
  GRBcblazy(void *cbdata, int lazylen, const int *lazyind,
            const double *lazyval, char lazysense, double lazyrhs);


/*
   ATTRIBUTES
*/

/* Model attributes */

#define GRB_INT_ATTR_NUMCONSTRS  "NumConstrs"  /* # of constraints */
#define GRB_INT_ATTR_NUMVARS     "NumVars"     /* # of vars */
#define GRB_INT_ATTR_NUMSOS      "NumSOS"      /* # of sos constraints */
#define GRB_INT_ATTR_NUMQCONSTRS "NumQConstrs" /* # of quadratic constraints */
#define GRB_INT_ATTR_NUMNZS      "NumNZs"      /* # of nz in A */
#define GRB_INT_ATTR_NUMQNZS     "NumQNZs"     /* # of nz in Q */
#define GRB_INT_ATTR_NUMQCNZS    "NumQCNZs"    /* # of nz in q constraints */
#define GRB_INT_ATTR_NUMINTVARS  "NumIntVars"  /* # of integer vars */
#define GRB_INT_ATTR_NUMBINVARS  "NumBinVars"  /* # of binary vars */
#define GRB_STR_ATTR_MODELNAME   "ModelName"   /* model name */
#define GRB_INT_ATTR_MODELSENSE  "ModelSense"  /* 1=min, -1=max */
#define GRB_DBL_ATTR_OBJCON      "ObjCon"      /* Objective constant */
#define GRB_INT_ATTR_IS_MIP      "IsMIP"       /* Is model a MIP? */
#define GRB_INT_ATTR_IS_QP       "IsQP"        /* Model has quadratic obj? */
#define GRB_INT_ATTR_IS_QCP      "IsQCP"       /* Model has quadratic constr? */

/* Variable attributes */

#define GRB_DBL_ATTR_LB             "LB"              /* Lower bound */
#define GRB_DBL_ATTR_UB             "UB"              /* Upper bound */
#define GRB_DBL_ATTR_OBJ            "Obj"             /* Objective coeff */
#define GRB_CHAR_ATTR_VTYPE         "VType"           /* Integrality type */
#define GRB_DBL_ATTR_START          "Start"           /* MIP start value */
#define GRB_DBL_ATTR_PSTART         "PStart"          /* LP primal solution warm start */
#define GRB_INT_ATTR_BRANCHPRIORITY "BranchPriority"  /* MIP branch priority */
#define GRB_STR_ATTR_VARNAME        "VarName"         /* Variable name */

/* Constraint attributes */

#define GRB_DBL_ATTR_RHS        "RHS"        /* RHS */
#define GRB_DBL_ATTR_DSTART     "DStart"     /* LP dual solution warm start */
#define GRB_CHAR_ATTR_SENSE     "Sense"      /* Sense ('<', '>', or '=') */
#define GRB_STR_ATTR_CONSTRNAME "ConstrName" /* Constraint name */

/* Quadratic constraint attributes */

#define GRB_DBL_ATTR_QCRHS    "QCRHS"   /* QC RHS */
#define GRB_CHAR_ATTR_QCSENSE "QCSense" /* QC sense ('<', '>', or '=') */
#define GRB_STR_ATTR_QCNAME   "QCName"  /* QC name */

/* Model statistics */

#define GRB_DBL_ATTR_MAX_COEFF     "MaxCoeff"    /* Max (abs) nz coeff in A */
#define GRB_DBL_ATTR_MIN_COEFF     "MinCoeff"    /* Min (abs) nz coeff in A */
#define GRB_DBL_ATTR_MAX_BOUND     "MaxBound"    /* Max (abs) finite var bd */
#define GRB_DBL_ATTR_MIN_BOUND     "MinBound"    /* Min (abs) var bd */
#define GRB_DBL_ATTR_MAX_OBJ_COEFF "MaxObjCoeff" /* Max (abs) obj coeff */
#define GRB_DBL_ATTR_MIN_OBJ_COEFF "MinObjCoeff" /* Min (abs) obj coeff */
#define GRB_DBL_ATTR_MAX_RHS       "MaxRHS"      /* Max (abs) rhs coeff */
#define GRB_DBL_ATTR_MIN_RHS       "MinRHS"      /* Min (abs) rhs coeff */

/* Model solution attributes */

#define GRB_DBL_ATTR_RUNTIME      "Runtime"      /* Run time for optimization */
#define GRB_INT_ATTR_STATUS       "Status"       /* Optimization status */
#define GRB_DBL_ATTR_OBJVAL       "ObjVal"       /* Solution objective */
#define GRB_DBL_ATTR_OBJBOUND     "ObjBound"     /* Best bound on solution */
#define GRB_INT_ATTR_SOLCOUNT     "SolCount"     /* # of solutions found */
#define GRB_DBL_ATTR_ITERCOUNT    "IterCount"    /* Iters performed (simplex) */
#define GRB_INT_ATTR_BARITERCOUNT "BarIterCount" /* Iters performed (barrier) */
#define GRB_DBL_ATTR_NODECOUNT    "NodeCount"    /* Nodes explored (B&C) */
#define GRB_INT_ATTR_HASDUALNORM  "HasDualNorm"  /* 0, no basis,
                                                    1, has basis, so can be computed
                                                    2, available */

/* Variable attributes related to the current solution */

#define GRB_DBL_ATTR_X         "X"         /* Solution value */
#define GRB_DBL_ATTR_Xn        "Xn"        /* Alternate MIP solution */
#define GRB_DBL_ATTR_RC        "RC"        /* Reduced costs */
#define GRB_DBL_ATTR_VDUALNORM "VDualNorm" /* Dual norm square */
#define GRB_INT_ATTR_VBASIS    "VBasis"    /* Variable basis status */

/* Constraint attributes related to the current solution */

#define GRB_DBL_ATTR_PI        "Pi"        /* Dual value */
#define GRB_DBL_ATTR_QCPI      "QCPi"      /* Dual value for QC */
#define GRB_DBL_ATTR_SLACK     "Slack"     /* Constraint slack */
#define GRB_DBL_ATTR_QCSLACK   "QCSlack"   /* QC Constraint slack */
#define GRB_DBL_ATTR_CDUALNORM "CDualNorm" /* Dual norm square */
#define GRB_INT_ATTR_CBASIS    "CBasis"    /* Constraint basis status */

/* Solution quality attributes */

#define GRB_DBL_ATTR_BOUND_VIO              "BoundVio"
#define GRB_DBL_ATTR_BOUND_SVIO             "BoundSVio"
#define GRB_INT_ATTR_BOUND_VIO_INDEX        "BoundVioIndex"
#define GRB_INT_ATTR_BOUND_SVIO_INDEX       "BoundSVioIndex"
#define GRB_DBL_ATTR_BOUND_VIO_SUM          "BoundVioSum"
#define GRB_DBL_ATTR_BOUND_SVIO_SUM         "BoundSVioSum"
#define GRB_DBL_ATTR_CONSTR_VIO             "ConstrVio"
#define GRB_DBL_ATTR_CONSTR_SVIO            "ConstrSVio"
#define GRB_INT_ATTR_CONSTR_VIO_INDEX       "ConstrVioIndex"
#define GRB_INT_ATTR_CONSTR_SVIO_INDEX      "ConstrSVioIndex"
#define GRB_DBL_ATTR_CONSTR_VIO_SUM         "ConstrVioSum"
#define GRB_DBL_ATTR_CONSTR_SVIO_SUM        "ConstrSVioSum"
#define GRB_DBL_ATTR_CONSTR_RESIDUAL        "ConstrResidual"
#define GRB_DBL_ATTR_CONSTR_SRESIDUAL       "ConstrSResidual"
#define GRB_INT_ATTR_CONSTR_RESIDUAL_INDEX  "ConstrResidualIndex"
#define GRB_INT_ATTR_CONSTR_SRESIDUAL_INDEX "ConstrSResidualIndex"
#define GRB_DBL_ATTR_CONSTR_RESIDUAL_SUM    "ConstrResidualSum"
#define GRB_DBL_ATTR_CONSTR_SRESIDUAL_SUM   "ConstrSResidualSum"
#define GRB_DBL_ATTR_DUAL_VIO               "DualVio"
#define GRB_DBL_ATTR_DUAL_SVIO              "DualSVio"
#define GRB_INT_ATTR_DUAL_VIO_INDEX         "DualVioIndex"
#define GRB_INT_ATTR_DUAL_SVIO_INDEX        "DualSVioIndex"
#define GRB_DBL_ATTR_DUAL_VIO_SUM           "DualVioSum"
#define GRB_DBL_ATTR_DUAL_SVIO_SUM          "DualSVioSum"
#define GRB_DBL_ATTR_DUAL_RESIDUAL          "DualResidual"
#define GRB_DBL_ATTR_DUAL_SRESIDUAL         "DualSResidual"
#define GRB_INT_ATTR_DUAL_RESIDUAL_INDEX    "DualResidualIndex"
#define GRB_INT_ATTR_DUAL_SRESIDUAL_INDEX   "DualSResidualIndex"
#define GRB_DBL_ATTR_DUAL_RESIDUAL_SUM      "DualResidualSum"
#define GRB_DBL_ATTR_DUAL_SRESIDUAL_SUM     "DualSResidualSum"
#define GRB_DBL_ATTR_INT_VIO                "IntVio"
#define GRB_INT_ATTR_INT_VIO_INDEX          "IntVioIndex"
#define GRB_DBL_ATTR_INT_VIO_SUM            "IntVioSum"
#define GRB_DBL_ATTR_COMPL_VIO              "ComplVio"
#define GRB_INT_ATTR_COMPL_VIO_INDEX        "ComplVioIndex"
#define GRB_DBL_ATTR_COMPL_VIO_SUM          "ComplVioSum"
#define GRB_DBL_ATTR_KAPPA                  "Kappa"
#define GRB_DBL_ATTR_KAPPA_EXACT            "KappaExact"
#define GRB_DBL_ATTR_N2KAPPA                "N2Kappa"

/* LP sensitivity analysis */

#define GRB_DBL_ATTR_SA_OBJLOW "SAObjLow"
#define GRB_DBL_ATTR_SA_OBJUP  "SAObjUp"
#define GRB_DBL_ATTR_SA_LBLOW  "SALBLow"
#define GRB_DBL_ATTR_SA_LBUP   "SALBUp"
#define GRB_DBL_ATTR_SA_UBLOW  "SAUBLow"
#define GRB_DBL_ATTR_SA_UBUP   "SAUBUp"
#define GRB_DBL_ATTR_SA_RHSLOW "SARHSLow"
#define GRB_DBL_ATTR_SA_RHSUP  "SARHSUp"

/* IIS */

#define GRB_INT_ATTR_IIS_MINIMAL "IISMinimal" /* Boolean: Is IIS Minimal? */
#define GRB_INT_ATTR_IIS_LB      "IISLB"      /* Boolean: Is var LB in IIS? */
#define GRB_INT_ATTR_IIS_UB      "IISUB"      /* Boolean: Is var UB in IIS? */
#define GRB_INT_ATTR_IIS_CONSTR  "IISConstr"  /* Boolean: Is constr in IIS? */
#define GRB_INT_ATTR_IIS_SOS     "IISSOS"     /* Boolean: Is SOS in IIS? */
#define GRB_INT_ATTR_IIS_QCONSTR "IISQConstr" /* Boolean: Is QConstr in IIS? */

/* advanced simplex features */

#define GRB_DBL_ATTR_FARKASDUAL  "FarkasDual"
#define GRB_DBL_ATTR_FARKASPROOF "FarkasProof"
#define GRB_DBL_ATTR_UNBDRAY     "UnbdRay"
#define GRB_INT_ATTR_INFEASVAR   "InfeasVar"
#define GRB_INT_ATTR_UNBDVAR     "UnbdVar"

/*
   CALLBACKS
*/

/* For callback */

#define GRB_CB_POLLING   0
#define GRB_CB_PRESOLVE  1
#define GRB_CB_SIMPLEX   2
#define GRB_CB_MIP       3
#define GRB_CB_MIPSOL    4
#define GRB_CB_MIPNODE   5
#define GRB_CB_MESSAGE   6
#define GRB_CB_BARRIER   7

/* Supported names for callback */

#define GRB_CB_PRE_COLDEL  1000
#define GRB_CB_PRE_ROWDEL  1001
#define GRB_CB_PRE_SENCHG  1002
#define GRB_CB_PRE_BNDCHG  1003
#define GRB_CB_PRE_COECHG  1004

#define GRB_CB_SPX_ITRCNT  2000
#define GRB_CB_SPX_OBJVAL  2001
#define GRB_CB_SPX_PRIMINF 2002
#define GRB_CB_SPX_DUALINF 2003
#define GRB_CB_SPX_ISPERT  2004

#define GRB_CB_MIP_OBJBST  3000
#define GRB_CB_MIP_OBJBND  3001
#define GRB_CB_MIP_NODCNT  3002
#define GRB_CB_MIP_SOLCNT  3003
#define GRB_CB_MIP_CUTCNT  3004
#define GRB_CB_MIP_NODLFT  3005
#define GRB_CB_MIP_ITRCNT  3006

#define GRB_CB_MIPSOL_SOL    4001
#define GRB_CB_MIPSOL_OBJ    4002
#define GRB_CB_MIPSOL_OBJBST 4003
#define GRB_CB_MIPSOL_OBJBND 4004
#define GRB_CB_MIPSOL_NODCNT 4005
#define GRB_CB_MIPSOL_SOLCNT 4006

#define GRB_CB_MIPNODE_STATUS 5001
#define GRB_CB_MIPNODE_REL    5002
#define GRB_CB_MIPNODE_OBJBST 5003
#define GRB_CB_MIPNODE_OBJBND 5004
#define GRB_CB_MIPNODE_NODCNT 5005
#define GRB_CB_MIPNODE_SOLCNT 5006
#define GRB_CB_MIPNODE_BRVAR  5007

#define GRB_CB_MSG_STRING  6001
#define GRB_CB_RUNTIME     6002

#define GRB_CB_BARRIER_ITRCNT  7001
#define GRB_CB_BARRIER_PRIMOBJ 7002
#define GRB_CB_BARRIER_DUALOBJ 7003
#define GRB_CB_BARRIER_PRIMINF 7004
#define GRB_CB_BARRIER_DUALINF 7005
#define GRB_CB_BARRIER_COMPL   7006

#define GRB_FEASRELAX_LINEAR      0
#define GRB_FEASRELAX_QUADRATIC   1
#define GRB_FEASRELAX_CARDINALITY 2

int __stdcall
  GRBgetcoeff(GRBmodel *model, int constr, int var, double *valP);
int __stdcall
  GRBgetconstrs(GRBmodel *model, int *numnzP, int *cbeg,
                int *cind, double *cval, int start, int len);
int __stdcall
  GRBgetvars(GRBmodel *model, int *numnzP, int *vbeg, int *vind,
             double *vval, int start, int len);
int __stdcall
  GRBgetsos(GRBmodel *model, int *nummembersP, int *sostype, int *beg,
            int *ind, double *weight, int start, int len);
int __stdcall
  GRBgetq(GRBmodel *model, int *numqnzP, int *qrow, int *qcol, double *qval);
int __stdcall
  GRBgetqconstr(GRBmodel *model, int qconstr,
                int *numlnzP, int *lind, double *lval,
                int *numqnzP, int *qrow, int *qcol, double *qval);
int __stdcall
  GRBgetvarbyname(GRBmodel *model, const char *name, int *indexP);
int __stdcall
  GRBgetconstrbyname(GRBmodel *model, const char *name, int *indexP);

int __stdcall
  GRBoptimize(GRBmodel *model);

GRBmodel * __stdcall
  GRBcopymodel(GRBmodel *model);
GRBmodel * __stdcall
  GRBfixedmodel(GRBmodel *model);
int __stdcall
  GRBfeasrelax(GRBmodel *model, int relaxobjtype, int minrelax,
               double *lbpen, double *ubpen, double *rhspen,
               double *feasobjP);

/* Undocumented routines */

int __stdcall
  GRBgetcbwhatinfo(void *cbdata, int what, int *typeP, int *sizeP);
GRBmodel * __stdcall
  GRBrelaxmodel(GRBmodel *model);
int __stdcall
  GRBconverttofixed(GRBmodel *lp);
GRBmodel * __stdcall
  GRBpresolvemodel(GRBmodel *model);
GRBmodel * __stdcall
  GRBiismodel(GRBmodel *model);
GRBmodel * __stdcall
  GRBfeasibility(GRBmodel *model);
GRBmodel * __stdcall
  GRBlinearizemodel(GRBmodel *model);
/* Copyright 2011, Gurobi Optimization, Inc. */

int __stdcall
  GRBreadmodel(GRBenv *env, const char *filename, GRBmodel **modelP);
int __stdcall
  GRBread(GRBmodel *model, const char *filename);
int __stdcall
  GRBwrite(GRBmodel *model, const char *filename);

int __stdcall
  GRBnewmodel(GRBenv *env, GRBmodel **modelP, const char *Pname, int numvars,
              double *obj, double *lb, double *ub, char *vtype,
              char **varnames);

int __stdcall
  GRBloadmodel(GRBenv *env, GRBmodel **modelP, const char *Pname,
               int numvars, int numconstrs,
               int objsense, double objcon, double *obj,
               char *sense, double *rhs,
               int *vbeg, int *vlen, int *vind, double *vval,
               double *lb, double *ub, char *vtype,
               char **varnames, char **constrnames);

int __stdcall
  GRBaddvar(GRBmodel *model, int numnz, int *vind, double *vval,
             double obj, double lb, double ub, char vtype,
             const char *varname);
int __stdcall
  GRBaddvars(GRBmodel *model, int numvars, int numnz,
             int *vbeg, int *vind, double *vval,
             double *obj, double *lb, double *ub, char *vtype,
             char **varnames);
int __stdcall
  GRBaddconstr(GRBmodel *model, int numnz, int *cind, double *cval,
               char sense, double rhs, const char *constrname);
int __stdcall
  GRBaddconstrs(GRBmodel *model, int numconstrs, int numnz,
                int *cbeg, int *cind, double *cval,
                char *sense, double *rhs, char **constrnames);
int __stdcall
  GRBaddrangeconstr(GRBmodel *model, int numnz, int *cind, double *cval,
                    double lower, double upper, const char *constrname);
int __stdcall
  GRBaddrangeconstrs(GRBmodel *model, int numconstrs, int numnz,
                     int *cbeg, int *cind, double *cval,
                     double *lower, double *upper, char **constrnames);
int __stdcall
  GRBaddsos(GRBmodel *model, int numsos, int nummembers, int *types,
            int *beg, int *ind, double *weight);
int __stdcall
  GRBaddqconstr(GRBmodel *model, int numlnz, int *lind, double *lval,
                int numqnz, int *qrow, int *qcol, double *qval,
                char sense, double rhs, const char *QCname);
int __stdcall
  GRBaddcone(GRBmodel *model, int nummembers, int *members);
int __stdcall
  GRBaddqpterms(GRBmodel *model, int numqnz, int *qrow, int *qcol,
                double *qval);
int __stdcall
  GRBdelvars(GRBmodel *model, int len, int *ind);
int __stdcall
  GRBdelconstrs(GRBmodel *model, int len, int *ind);
int __stdcall
  GRBdelsos(GRBmodel *model, int len, int *ind);
int __stdcall
  GRBdelqconstrs(GRBmodel *model, int len, int *ind);
int __stdcall
  GRBdelq(GRBmodel *model);
int __stdcall
  GRBchgcoeffs(GRBmodel *model, int cnt, int *cind, int *vind, double *val);

int __stdcall
  GRBupdatemodel(GRBmodel *model);

int __stdcall
  GRBresetmodel(GRBmodel *model);

int __stdcall
  GRBfreemodel(GRBmodel *model);

int __stdcall
  GRBcomputeIIS(GRBmodel *lp);

/* simplex advanced routines */

typedef struct SVector
{
  int     len;
  int    *ind;
  double *val;
} SVECTOR;

int __stdcall
GRBFSolve(GRBmodel *lp, SVECTOR *b, SVECTOR *x);

int __stdcall
GRBBinvColj(GRBmodel *lp, int j, SVECTOR *x);

int __stdcall
GRBBinvj(GRBmodel *lp, int j, SVECTOR *x);

int __stdcall
GRBBSolve(GRBmodel *lp, SVECTOR *b, SVECTOR *x);

int __stdcall
GRBBinvi(GRBmodel *lp, int i, SVECTOR *x);

int __stdcall
GRBBinvRowi(GRBmodel *lp, int i, SVECTOR *x);

int __stdcall
GRBgetBasisHead(GRBmodel *lp, int *bhead);

/* Model status codes (after call to GRBoptimize()) */

#define GRB_LOADED          1
#define GRB_OPTIMAL         2
#define GRB_INFEASIBLE      3
#define GRB_INF_OR_UNBD     4
#define GRB_UNBOUNDED       5
#define GRB_CUTOFF          6
#define GRB_ITERATION_LIMIT 7
#define GRB_NODE_LIMIT      8
#define GRB_TIME_LIMIT      9
#define GRB_SOLUTION_LIMIT 10
#define GRB_INTERRUPTED    11
#define GRB_NUMERIC        12
#define GRB_SUBOPTIMAL     13

/* Basis status info */

#define GRB_BASIC           0
#define GRB_NONBASIC_LOWER -1
#define GRB_NONBASIC_UPPER -2
#define GRB_SUPERBASIC     -3

/* Copyright 2011, Gurobi Optimization, Inc. */

/* Undocumented routines */

int __stdcall
GRBstrongbranch(GRBmodel *model, int num, int *cand,
                double *downobjbd, double *upobjbd, int *statusP);

/* Copyright 2011, Gurobi Optimization, Inc. */

/**************/
/* Parameters */
/**************/

/* Termination */

#define GRB_INT_PAR_BARITERLIMIT   "BarIterLimit"
#define GRB_DBL_PAR_CUTOFF         "Cutoff"
#define GRB_DBL_PAR_ITERATIONLIMIT "IterationLimit"
#define GRB_DBL_PAR_NODELIMIT      "NodeLimit"
#define GRB_INT_PAR_SOLUTIONLIMIT  "SolutionLimit"
#define GRB_DBL_PAR_TIMELIMIT      "TimeLimit"

/* Tolerances */

#define GRB_DBL_PAR_FEASIBILITYTOL "FeasibilityTol"
#define GRB_DBL_PAR_INTFEASTOL     "IntFeasTol"
#define GRB_DBL_PAR_MARKOWITZTOL   "MarkowitzTol"
#define GRB_DBL_PAR_MIPGAP         "MIPGap"
#define GRB_DBL_PAR_MIPGAPABS      "MIPGapAbs"
#define GRB_DBL_PAR_OPTIMALITYTOL  "OptimalityTol"
#define GRB_DBL_PAR_PSDTOL         "PSDTol"

/* Simplex */

#define GRB_INT_PAR_METHOD         "Method"
#define GRB_DBL_PAR_PERTURBVALUE   "PerturbValue"
#define GRB_DBL_PAR_OBJSCALE       "ObjScale"
#define GRB_INT_PAR_SCALEFLAG      "ScaleFlag"
#define GRB_INT_PAR_SIMPLEXPRICING "SimplexPricing"
#define GRB_INT_PAR_QUAD           "Quad"
#define GRB_INT_PAR_NORMADJUST     "NormAdjust"
#define GRB_INT_PAR_SIFTING        "Sifting"
#define GRB_INT_PAR_SIFTMETHOD     "SiftMethod"

/* Barrier */

#define GRB_DBL_PAR_BARCONVTOL     "BarConvTol"
#define GRB_INT_PAR_BARCORRECTORS  "BarCorrectors"
#define GRB_INT_PAR_BARHOMOGENEOUS "BarHomogeneous"
#define GRB_INT_PAR_BARORDER       "BarOrder"
#define GRB_DBL_PAR_BARQCPCONVTOL  "BarQCPConvTol"
#define GRB_INT_PAR_CROSSOVER      "Crossover"
#define GRB_INT_PAR_CROSSOVERBASIS "CrossoverBasis"

/* MIP */

#define GRB_INT_PAR_BRANCHDIR         "BranchDir"
#define GRB_DBL_PAR_HEURISTICS        "Heuristics"
#define GRB_DBL_PAR_IMPROVESTARTGAP   "ImproveStartGap"
#define GRB_DBL_PAR_IMPROVESTARTTIME  "ImproveStartTime"
#define GRB_DBL_PAR_IMPROVESTARTNODES "ImproveStartNodes"
#define GRB_INT_PAR_MINRELNODES       "MinRelNodes"
#define GRB_INT_PAR_MIPFOCUS          "MIPFocus"
#define GRB_STR_PAR_NODEFILEDIR       "NodefileDir"
#define GRB_DBL_PAR_NODEFILESTART     "NodefileStart"
#define GRB_INT_PAR_NODEMETHOD        "NodeMethod"
#define GRB_INT_PAR_PUMPPASSES        "PumpPasses"
#define GRB_INT_PAR_RINS              "RINS"
#define GRB_INT_PAR_SUBMIPNODES       "SubMIPNodes"
#define GRB_INT_PAR_SYMMETRY          "Symmetry"
#define GRB_INT_PAR_VARBRANCH         "VarBranch"
#define GRB_INT_PAR_SOLUTIONNUMBER    "SolutionNumber"
#define GRB_INT_PAR_ZEROOBJNODES      "ZeroObjNodes"

/* MIP cuts */

#define GRB_INT_PAR_CUTS          "Cuts"

#define GRB_INT_PAR_CLIQUECUTS    "CliqueCuts"
#define GRB_INT_PAR_COVERCUTS     "CoverCuts"
#define GRB_INT_PAR_FLOWCOVERCUTS "FlowCoverCuts"
#define GRB_INT_PAR_FLOWPATHCUTS  "FlowPathCuts"
#define GRB_INT_PAR_GUBCOVERCUTS  "GUBCoverCuts"
#define GRB_INT_PAR_IMPLIEDCUTS   "ImpliedCuts"
#define GRB_INT_PAR_MIPSEPCUTS    "MIPSepCuts"
#define GRB_INT_PAR_MIRCUTS       "MIRCuts"
#define GRB_INT_PAR_MODKCUTS      "ModKCuts"
#define GRB_INT_PAR_ZEROHALFCUTS  "ZeroHalfCuts"
#define GRB_INT_PAR_NETWORKCUTS   "NetworkCuts"
#define GRB_INT_PAR_SUBMIPCUTS    "SubMIPCuts"

#define GRB_INT_PAR_CUTAGGPASSES  "CutAggPasses"
#define GRB_INT_PAR_CUTPASSES     "CutPasses"
#define GRB_INT_PAR_GOMORYPASSES  "GomoryPasses"

/* Other */

#define GRB_INT_PAR_AGGREGATE       "Aggregate"
#define GRB_INT_PAR_AGGFILL         "AggFill"
#define GRB_INT_PAR_DISPLAYINTERVAL "DisplayInterval"
#define GRB_INT_PAR_DUALREDUCTIONS  "DualReductions"
#define GRB_INT_PAR_IISMETHOD       "IISMethod"
#define GRB_INT_PAR_INFUNBDINFO     "InfUnbdInfo"
#define GRB_STR_PAR_LOGFILE         "LogFile"
#define GRB_INT_PAR_LOGTOCONSOLE    "LogToConsole"
#define GRB_INT_PAR_MIQCPMETHOD     "MIQCPMethod"
#define GRB_INT_PAR_OUTPUTFLAG      "OutputFlag"
#define GRB_INT_PAR_PRECRUSH        "PreCrush"
#define GRB_INT_PAR_PREDEPROW       "PreDepRow"
#define GRB_INT_PAR_PREDUAL         "PreDual"
#define GRB_INT_PAR_PREPASSES       "PrePasses"
#define GRB_INT_PAR_PREQLINEARIZE   "PreQLinearize"
#define GRB_INT_PAR_PRESOLVE        "Presolve"
#define GRB_INT_PAR_PRESPARSIFY     "PreSparsify"
#define GRB_INT_PAR_QCPDUAL         "QCPDual"
#define GRB_STR_PAR_RESULTFILE      "ResultFile"
#define GRB_INT_PAR_SEED            "Seed"
#define GRB_INT_PAR_THREADS         "Threads"
#define GRB_DBL_PAR_FEASRELAXBIGM   "FeasRelaxBigM"
#define GRB_STR_PAR_DUMMY           "Dummy"

/* Parameter enumerations */

/* All *CUTS parameters */
#define GRB_CUTS_AUTO          -1
#define GRB_CUTS_OFF            0
#define GRB_CUTS_CONSERVATIVE   1
#define GRB_CUTS_AGGRESSIVE     2
#define GRB_CUTS_VERYAGGRESSIVE 3

#define GRB_PRESOLVE_AUTO        -1
#define GRB_PRESOLVE_OFF          0
#define GRB_PRESOLVE_CONSERVATIVE 1
#define GRB_PRESOLVE_AGGRESSIVE   2

#define GRB_METHOD_AUTO                    -1
#define GRB_METHOD_PRIMAL                   0
#define GRB_METHOD_DUAL                     1
#define GRB_METHOD_BARRIER                  2
#define GRB_METHOD_CONCURRENT               3
#define GRB_METHOD_DETERMINISTIC_CONCURRENT 4

#define GRB_BARHOMOGENEOUS_AUTO -1
#define GRB_BARHOMOGENEOUS_OFF   0
#define GRB_BARHOMOGENEOUS_ON    1

#define GRB_MIPFOCUS_BALANCED    0
#define GRB_MIPFOCUS_FEASIBILITY 1
#define GRB_MIPFOCUS_OPTIMALITY  2
#define GRB_MIPFOCUS_BESTBOUND   3

#define GRB_BARORDER_AUTOMATIC       -1
#define GRB_BARORDER_AMD              0
#define GRB_BARORDER_NESTEDDISSECTION 1

#define GRB_SIMPLEXPRICING_AUTO           -1
#define GRB_SIMPLEXPRICING_PARTIAL         0
#define GRB_SIMPLEXPRICING_STEEPEST_EDGE   1
#define GRB_SIMPLEXPRICING_DEVEX           2
#define GRB_SIMPLEXPRICING_STEEPEST_QUICK  3

#define GRB_VARBRANCH_AUTO          -1
#define GRB_VARBRANCH_PSEUDO_REDUCED 0
#define GRB_VARBRANCH_PSEUDO_SHADOW  1
#define GRB_VARBRANCH_MAX_INFEAS     2
#define GRB_VARBRANCH_STRONG         3

int __stdcall
  GRBcheckmodel(GRBmodel *model);
void __stdcall
  GRBsetsignal(GRBmodel *model);
void __stdcall
  GRBterminate(GRBmodel *model);
void __stdcall
  GRBclean2(int *lenP, int *ind, double *val);
void __stdcall
  GRBclean3(int *lenP, int *ind0, int *ind1, double *val);

/* Logging */

void __stdcall
  GRBmsg(GRBenv *env, const char *message);


/* The following four routines are deprecated in Gurobi 2.0.
   Use the 'LogFile' parameter to control logging instead. */

int __stdcall
  GRBgetlogfile(GRBenv *env, FILE **logfileP);
int __stdcall
  GRBsetlogfile(GRBenv *env, FILE *logfile);


/* Parameter routines */

int __stdcall
  GRBgetintparam(GRBenv *env, const char *paramname, int *valueP);
int __stdcall
  GRBgetdblparam(GRBenv *env, const char *paramname, double *valueP);
int __stdcall
  GRBgetstrparam(GRBenv *env, const char *paramname, char *valueP);
int __stdcall
  GRBgetintparaminfo(GRBenv *env, const char *paramname, int *valueP,
                     int *minP, int *maxP, int *defP);
int __stdcall
  GRBgetdblparaminfo(GRBenv *env, const char *paramname, double *valueP,
                     double *minP, double *maxP, double *defP);
int __stdcall
  GRBgetstrparaminfo(GRBenv *env, const char *paramname, char *valueP,
                     char *defP);
int __stdcall
  GRBsetparam(GRBenv *env, const char *paramname, const char *value);
int __stdcall
  GRBsetintparam(GRBenv *env, const char *paramname, int value);
int __stdcall
  GRBsetdblparam(GRBenv *env, const char *paramname, double value);
int __stdcall
  GRBsetstrparam(GRBenv *env, const char *paramname, const char *value);
int __stdcall
  GRBgetparamtype(GRBenv *env, const char *paramname);
int __stdcall
  GRBresetparams(GRBenv *env);
int __stdcall
  GRBwriteparams(GRBenv *env, const char *filename);
int __stdcall
  GRBreadparams(GRBenv *env, const char *filename);
int __stdcall
  GRBgetnumparams(GRBenv *env);
int __stdcall
  GRBgetparamname(GRBenv *env, int i, char **paramnameP);

/* Environment routines */

int __stdcall
  GRBloadenv(GRBenv **envP, const char *logfilename);
GRBenv *__stdcall
  GRBgetenv(GRBmodel *model);
GRBenv *__stdcall
  GRBgetconcurrentenv(GRBmodel *model, int num);
void __stdcall
  GRBdiscardconcurrentenvs(GRBmodel *model);
void __stdcall
  GRBreleaselicense(GRBenv *env);
void __stdcall
  GRBfreeenv(GRBenv *env);
const char * __stdcall
  GRBgeterrormsg(GRBenv *env);
const char * __stdcall
  GRBgetmerrormsg(GRBmodel *model);

/* Version info */

void __stdcall
  GRBversion(int *majorP, int *minorP, int *technicalP);

char * __stdcall
  GRBplatform(void);

int __stdcall
  GRBlisttokens(void);

/* Deprecated */

#define GRB_INT_PAR_PREMIQPMETHOD   "PreMIQPMethod"
#endif

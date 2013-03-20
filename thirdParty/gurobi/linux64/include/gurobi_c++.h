/* Copyright 2013, Gurobi Optimization, Inc. */
 
#ifndef _GUROBI_CPP_H
#define _GUROBI_CPP_H
 
#include <iostream>
#include <string>
#include <assert.h>
#include <vector>
#include <fstream>

using std::string;
using std::vector;
using std::ostream;

extern "C" {
#include "gurobi_c.h"

int __stdcall
  GRBislp(GRBenv **envP, const char *logfilename, const char *isvname,
          const char *appname, const char *isv_key);
}

#define GRB_ERROR_NOT_IN_MODEL            20001
#define GRB_ERROR_FAILED_TO_CREATE_MODEL  20002
#define GRB_ERROR_INTERNAL                20003

enum GRB_DoubleParam {
  GRB_DoubleParam_Cutoff,
  GRB_DoubleParam_IterationLimit,
  GRB_DoubleParam_NodeLimit,
  GRB_DoubleParam_TimeLimit,
  GRB_DoubleParam_FeasibilityTol,
  GRB_DoubleParam_IntFeasTol,
  GRB_DoubleParam_MarkowitzTol,
  GRB_DoubleParam_MIPGap,
  GRB_DoubleParam_MIPGapAbs,
  GRB_DoubleParam_OptimalityTol,
  GRB_DoubleParam_PerturbValue,
  GRB_DoubleParam_Heuristics,
  GRB_DoubleParam_ObjScale,
  GRB_DoubleParam_NodefileStart,
  GRB_DoubleParam_BarConvTol,
  GRB_DoubleParam_BarQCPConvTol,
  GRB_DoubleParam_PSDTol,
  GRB_DoubleParam_ImproveStartGap,
  GRB_DoubleParam_ImproveStartTime,
  GRB_DoubleParam_FeasRelaxBigM
};

enum GRB_IntParam {
  GRB_IntParam_SolutionLimit,
  GRB_IntParam_Method,
  GRB_IntParam_LPMethod,
  GRB_IntParam_ScaleFlag,
  GRB_IntParam_SimplexPricing,
  GRB_IntParam_Quad,
  GRB_IntParam_NormAdjust,
  GRB_IntParam_Sifting,
  GRB_IntParam_SiftMethod,
  GRB_IntParam_SubMIPNodes,
  GRB_IntParam_VarBranch,
  GRB_IntParam_Cuts,
  GRB_IntParam_CliqueCuts,
  GRB_IntParam_CoverCuts,
  GRB_IntParam_FlowCoverCuts,
  GRB_IntParam_FlowPathCuts,
  GRB_IntParam_GUBCoverCuts,
  GRB_IntParam_ImpliedCuts,
  GRB_IntParam_MIPSepCuts,
  GRB_IntParam_MIRCuts,
  GRB_IntParam_ModKCuts,
  GRB_IntParam_ZeroHalfCuts,
  GRB_IntParam_NetworkCuts,
  GRB_IntParam_SubMIPCuts,
  GRB_IntParam_CutAggPasses,
  GRB_IntParam_CutPasses,
  GRB_IntParam_GomoryPasses,
  GRB_IntParam_NodeMethod,
  GRB_IntParam_Presolve,
  GRB_IntParam_Aggregate,
  GRB_IntParam_IISMethod,
  GRB_IntParam_PreCrush,
  GRB_IntParam_PreDepRow,
  GRB_IntParam_PreMIQPMethod,
  GRB_IntParam_PrePasses,
  GRB_IntParam_DisplayInterval,
  GRB_IntParam_OutputFlag,
  GRB_IntParam_Threads,
  GRB_IntParam_BarIterLimit,
  GRB_IntParam_Crossover,
  GRB_IntParam_CrossoverBasis,
  GRB_IntParam_BarCorrectors,
  GRB_IntParam_BarOrder,
  GRB_IntParam_PumpPasses,
  GRB_IntParam_RINS,
  GRB_IntParam_Symmetry,
  GRB_IntParam_MIPFocus,
  GRB_IntParam_AggFill,
  GRB_IntParam_PreDual,
  GRB_IntParam_SolutionNumber,
  GRB_IntParam_MinRelNodes,
  GRB_IntParam_ZeroObjNodes,
  GRB_IntParam_BranchDir,
  GRB_IntParam_InfUnbdInfo,
  GRB_IntParam_DualReductions,
  GRB_IntParam_BarHomogeneous,
  GRB_IntParam_PreQLinearize,
  GRB_IntParam_MIQCPMethod,
  GRB_IntParam_QCPDual,
  GRB_IntParam_LogToConsole,
  GRB_IntParam_PreSparsify
};

enum GRB_StringParam {
  GRB_StringParam_LogFile,
  GRB_StringParam_NodefileDir,
  GRB_StringParam_ResultFile,
  GRB_StringParam_Dummy
};

enum GRB_IntAttr {
  GRB_IntAttr_NumConstrs,
  GRB_IntAttr_NumVars,
  GRB_IntAttr_NumSOS,
  GRB_IntAttr_NumQConstrs,
  GRB_IntAttr_NumNZs,
  GRB_IntAttr_NumQNZs,
  GRB_IntAttr_NumQCNZs,
  GRB_IntAttr_NumIntVars,
  GRB_IntAttr_NumBinVars,
  GRB_IntAttr_ModelSense,
  GRB_IntAttr_IsMIP,
  GRB_IntAttr_IsQP,
  GRB_IntAttr_IsQCP,
  GRB_IntAttr_Status,
  GRB_IntAttr_SolCount,
  GRB_IntAttr_BarIterCount,
  GRB_IntAttr_VBasis,
  GRB_IntAttr_CBasis,
  GRB_IntAttr_BranchPriority,
  GRB_IntAttr_BoundVioIndex,
  GRB_IntAttr_BoundSVioIndex,
  GRB_IntAttr_ConstrVioIndex,
  GRB_IntAttr_ConstrSVioIndex,
  GRB_IntAttr_ConstrResidualIndex,
  GRB_IntAttr_ConstrSResidualIndex,
  GRB_IntAttr_DualVioIndex,
  GRB_IntAttr_DualSVioIndex,
  GRB_IntAttr_DualResidualIndex,
  GRB_IntAttr_DualSResidualIndex,
  GRB_IntAttr_ComplVioIndex,
  GRB_IntAttr_IntVioIndex,
  GRB_IntAttr_IISMinimal,
  GRB_IntAttr_IISLB,
  GRB_IntAttr_IISUB,
  GRB_IntAttr_IISConstr,
  GRB_IntAttr_IISSOS,
  GRB_IntAttr_IISQConstr
};

enum GRB_CharAttr {
  GRB_CharAttr_VType,
  GRB_CharAttr_Sense,
  GRB_CharAttr_QCSense
};

enum GRB_DoubleAttr {
  GRB_DoubleAttr_Runtime,
  GRB_DoubleAttr_ObjCon,
  GRB_DoubleAttr_LB,
  GRB_DoubleAttr_UB,
  GRB_DoubleAttr_Obj,
  GRB_DoubleAttr_Start,
  GRB_DoubleAttr_RHS,
  GRB_DoubleAttr_QCRHS,
  GRB_DoubleAttr_MaxCoeff,
  GRB_DoubleAttr_MinCoeff,
  GRB_DoubleAttr_MaxBound,
  GRB_DoubleAttr_MinBound,
  GRB_DoubleAttr_MaxObjCoeff,
  GRB_DoubleAttr_MinObjCoeff,
  GRB_DoubleAttr_MaxRHS,
  GRB_DoubleAttr_MinRHS,
  GRB_DoubleAttr_ObjVal,
  GRB_DoubleAttr_ObjBound,
  GRB_DoubleAttr_IterCount,
  GRB_DoubleAttr_NodeCount,
  GRB_DoubleAttr_X,
  GRB_DoubleAttr_RC,
  GRB_DoubleAttr_Pi,
  GRB_DoubleAttr_QCPi,
  GRB_DoubleAttr_Slack,
  GRB_DoubleAttr_QCSlack,
  GRB_DoubleAttr_BoundVio,
  GRB_DoubleAttr_BoundSVio,
  GRB_DoubleAttr_BoundVioSum,
  GRB_DoubleAttr_BoundSVioSum,
  GRB_DoubleAttr_ConstrVio,
  GRB_DoubleAttr_ConstrSVio,
  GRB_DoubleAttr_ConstrVioSum,
  GRB_DoubleAttr_ConstrSVioSum,
  GRB_DoubleAttr_ConstrResidual,
  GRB_DoubleAttr_ConstrSResidual,
  GRB_DoubleAttr_ConstrResidualSum,
  GRB_DoubleAttr_ConstrSResidualSum,
  GRB_DoubleAttr_DualVio,
  GRB_DoubleAttr_DualSVio,
  GRB_DoubleAttr_DualVioSum,
  GRB_DoubleAttr_DualSVioSum,
  GRB_DoubleAttr_DualResidual,
  GRB_DoubleAttr_DualSResidual,
  GRB_DoubleAttr_DualResidualSum,
  GRB_DoubleAttr_DualSResidualSum,
  GRB_DoubleAttr_ComplVio,
  GRB_DoubleAttr_ComplVioSum,
  GRB_DoubleAttr_IntVio,
  GRB_DoubleAttr_IntVioSum,
  GRB_DoubleAttr_Kappa,
  GRB_DoubleAttr_KappaExact,
  GRB_DoubleAttr_SAObjLow,
  GRB_DoubleAttr_SAObjUp,
  GRB_DoubleAttr_SALBLow,
  GRB_DoubleAttr_SALBUp,
  GRB_DoubleAttr_SARHSLow,
  GRB_DoubleAttr_SAUBLow,
  GRB_DoubleAttr_SAUBUp,
  GRB_DoubleAttr_SARHSUp,
  GRB_DoubleAttr_Xn,
  GRB_DoubleAttr_FarkasProof,
  GRB_DoubleAttr_FarkasDual,
  GRB_DoubleAttr_UnbdRay,
  GRB_DoubleAttr_PStart,
  GRB_DoubleAttr_DStart
};

enum GRB_StringAttr {
  GRB_StringAttr_ModelName,
  GRB_StringAttr_VarName,
  GRB_StringAttr_ConstrName,
  GRB_StringAttr_QCName
};

class GRBVar;
class GRBExpr;
class GRBLinExpr;
class GRBQuadExpr;
class GRBConstr;
class GRBModel;
class GRBEnv;
class GRBException;
class GRBCallback;
class GRBSOS;
class GRBQConstr;
class GRBColumn;
class GRBTempConstr;

ostream& operator<<(ostream &stream, GRBLinExpr expr);
GRBLinExpr operator+(const GRBLinExpr& x, const GRBLinExpr& y);
GRBLinExpr operator-(const GRBLinExpr& x, const GRBLinExpr& y);
GRBLinExpr operator+(const GRBLinExpr& x);
GRBLinExpr operator+(GRBVar x, GRBVar y);
GRBLinExpr operator+(GRBVar x, double a);
GRBLinExpr operator+(double a, GRBVar x);
GRBLinExpr operator-(const GRBLinExpr& x);
GRBLinExpr operator-(GRBVar x);
GRBLinExpr operator-(GRBVar x, GRBVar y);
GRBLinExpr operator-(GRBVar x, double a);
GRBLinExpr operator-(double a, GRBVar x);
GRBLinExpr operator*(double a, GRBVar x);
GRBLinExpr operator*(GRBVar x, double a);
GRBLinExpr operator*(const GRBLinExpr& x, double a);
GRBLinExpr operator*(double a, const GRBLinExpr& x);
GRBLinExpr operator/(GRBVar x, double a);
GRBLinExpr operator/(const GRBLinExpr& x, double a);

ostream& operator<<(ostream &stream, GRBQuadExpr expr);
GRBQuadExpr operator+(const GRBQuadExpr& x, const GRBQuadExpr& y);
GRBQuadExpr operator-(const GRBQuadExpr& x, const GRBQuadExpr& y);
GRBQuadExpr operator+(const GRBQuadExpr& x);
GRBQuadExpr operator-(const GRBQuadExpr& x);
GRBQuadExpr operator*(const GRBQuadExpr& x, double a);
GRBQuadExpr operator*(double a, const GRBQuadExpr& x);
GRBQuadExpr operator*(GRBVar x, GRBVar y);
GRBQuadExpr operator*(GRBVar x, const GRBLinExpr& y);
GRBQuadExpr operator*(const GRBLinExpr& y, GRBVar x);
GRBQuadExpr operator*(const GRBLinExpr& x, const GRBLinExpr& y);
GRBQuadExpr operator/(const GRBQuadExpr& x, double a);

GRBTempConstr operator<=(GRBQuadExpr x, GRBQuadExpr y);
GRBTempConstr operator>=(GRBQuadExpr x, GRBQuadExpr y);
GRBTempConstr operator==(GRBQuadExpr x, GRBQuadExpr y);

class GRBEnv
{
  private:

    GRBenv*  env;
    GRBenv** envP;

  public:

    friend class GRBModel;

    GRBEnv();
    GRBEnv(const string& logfilename);
    GRBEnv(const string&, const string&, const string&, const string&);
    ~GRBEnv();
    void message(const string& msg);
    int get(GRB_IntParam param) const;
    double get(GRB_DoubleParam param) const;
    const string get(GRB_StringParam param) const;
    void set(GRB_IntParam param, int newvalue);
    void set(GRB_DoubleParam param, double newvalue);
    void set(GRB_StringParam param, const string& newvalue);
    void getParamInfo(GRB_DoubleParam param, double* valP, 
                      double* minP, double* maxP, double* defP);
    void getParamInfo(GRB_IntParam param, int* valP, int* minP,
                      int* maxP, int* defP);
    void getParamInfo(GRB_StringParam param, string& value,
                      string& defvalue);
    void resetParams();
    void writeParams(const string& paramfile);
    void readParams(const string& paramfile);
    const string getErrorMsg() const;
};

class GRBModel
{
  private:

    GRBmodel    *Cmodel;
    GRBenv      *Cenv;
    GRBCallback *cb;

    int rows;
    int cols;
    int numsos;
    int numqconstrs;

    vector<GRBVar> vars;
    vector<GRBConstr> constrs;
    vector<GRBSOS> sos;
    vector<GRBQConstr> qconstrs;


  public:

    GRBModel(const GRBEnv& env);
    GRBModel(const GRBEnv& env, const string& filename);
    GRBModel(const GRBModel& xmodel);

    ~GRBModel();

    void read(const string& filename);
    void write(const string& filename);

    GRBModel relax();
    GRBModel fixedModel();
    GRBModel presolve();
    GRBModel feasibility();

    double feasRelax(int relaxobjtype, bool minrelax, bool vrelax, bool crelax);
    double feasRelax(int relaxobjtype, bool minrelax, int vlen,
                   const GRBVar* vars, const double* lbpen,
                   const double* ubpen, int clen, const GRBConstr* constrs,
                   const double* rhspen);

    void update();
    void optimize();
    void computeIIS();
    void reset();
    void check();
    void terminate();

    GRBQuadExpr getObjective() const;
    void setObjective(GRBLinExpr obje, int sense=0);
    void setObjective(GRBQuadExpr obje, int sense=0);

    GRBVar getVar(int i) const;
    GRBVar* getVars() const;
    GRBVar getVarByName(const string& name);
    GRBConstr getConstr(int i) const;
    GRBConstr* getConstrs() const;
    GRBConstr getConstrByName(const string& name);
    GRBSOS* getSOSs() const;
    GRBQConstr* getQConstrs() const;

    GRBVar addVar(double lb, double ub, double obj, char vtype,
                  string vname="");
    GRBVar addVar(double lb, double ub, double obj, char vtype,
                  int nonzeros, const GRBConstr* xconstrs,
                  const double* coeffs=NULL, string name="");
    GRBVar addVar(double lb, double ub, double obj, char vtype,
                  const GRBColumn& col, string name="");
    GRBVar* addVars(int cnt, char type=GRB_CONTINUOUS);
    GRBVar* addVars(const double* lb, const double* ub,
                    const double* obj, const char* type,
                    const string* name, int len);
    GRBVar* addVars(const double* lb, const double *ub,
                    const double* obj, const char* type,
                    const string* name, const GRBColumn*
                    col, int len);

    GRBConstr addConstr(const GRBLinExpr& expr1, char sense,
                        const GRBLinExpr& expr2,
                        string name="");
    GRBConstr addConstr(const GRBLinExpr& expr, char sense, GRBVar v,
                        string name="");
    GRBConstr addConstr(GRBVar v1, char sense, GRBVar v2,
                        string name="");
    GRBConstr addConstr(GRBVar v, char sense, double rhs,
                        string name="");
    GRBConstr addConstr(const GRBLinExpr& expr, char sense, double rhs,
                        string name="");
    GRBConstr addConstr(const GRBTempConstr& tc, string name="");
    GRBConstr addRange(const GRBLinExpr& expr, double lower, double upper,
                       string name="");
    GRBConstr* addConstrs(int cnt);
    GRBConstr* addConstrs(const GRBLinExpr* expr, const char* sense,
                          const double* rhs, const string* name,
                          int len);
    GRBConstr* addRanges(const GRBLinExpr* expr, const double* lower,
                         const double* upper, const string* name,
                         int len);
    GRBSOS addSOS(const GRBVar* xvars, const double* weight, int len, int type);
    GRBQConstr addQConstr(const GRBQuadExpr& expr1, char sense,
                          const GRBQuadExpr& expr2,
                          string name="");
    GRBQConstr addQConstr(const GRBTempConstr& tc, string name="");
    GRBQConstr addQConstr(const GRBQuadExpr&  expr, char sense, double rhs,
                          string cname);

    void remove(GRBVar v);
    void remove(GRBConstr c);
    void remove(GRBSOS xsos);
    void remove(GRBQConstr xqconstr);

    void chgCoeff(GRBConstr c, GRBVar v, double val);
    void chgCoeffs(const GRBConstr* xconstrs, const GRBVar* xvars,
                   const double* val, int len);
    double getCoeff(GRBConstr c, GRBVar v) const;
    GRBColumn getCol(GRBVar v);
    GRBLinExpr getRow(GRBConstr c);
    int getSOS(GRBSOS xsos, GRBVar* xvars, double* weight, int* typeP);
    GRBQuadExpr getQCRow(GRBQConstr c);
    GRBEnv getEnv();
    GRBEnv getConcurrentEnv(int num);
    void discardConcurrentEnvs();

    int    get(GRB_IntAttr attr) const;
    double get(GRB_DoubleAttr attr) const;
    string get(GRB_StringAttr attr) const;

    void set(GRB_IntAttr attr, int val);
    void set(GRB_DoubleAttr attr, double val);
    void set(GRB_StringAttr attr, const string& val);

    int*    get(GRB_IntAttr    attr, const GRBVar* xvars, int len);
    char*   get(GRB_CharAttr   attr, const GRBVar* xvars, int len);
    double* get(GRB_DoubleAttr attr, const GRBVar* xvars, int len);
    string* get(GRB_StringAttr attr, const GRBVar* xvars, int len);

    int*    get(GRB_IntAttr    attr, const GRBConstr* xconstrs, int len);
    char*   get(GRB_CharAttr   attr, const GRBConstr* xconstrs, int len);
    double* get(GRB_DoubleAttr attr, const GRBConstr* xconstrs, int len);
    string* get(GRB_StringAttr attr, const GRBConstr* xconstrs, int len);

    char*   get(GRB_CharAttr   attr, const GRBQConstr* xqconstrs, int len);
    double* get(GRB_DoubleAttr attr, const GRBQConstr* xqconstrs, int len);
    string* get(GRB_StringAttr attr, const GRBQConstr* xqconstrs, int len);

    void set(GRB_IntAttr    attr, const GRBVar* xvars,
             const int*    val, int len);
    void set(GRB_CharAttr   attr, const GRBVar* xvars,
             const char*   val, int len);
    void set(GRB_DoubleAttr attr, const GRBVar* xvars,
             const double* val, int len);
    void set(GRB_StringAttr attr, const GRBVar* xvars,
             const string* val, int len);

    void set(GRB_IntAttr    attr, const GRBConstr* xconstrs,
             const int*    val, int len);
    void set(GRB_CharAttr   attr, const GRBConstr* xconstrs,
             const char*   val, int len);
    void set(GRB_DoubleAttr attr, const GRBConstr* xconstrs,
             const double* val, int len);
    void set(GRB_StringAttr attr, const GRBConstr* xconstrs,
             const string* val, int len);

    void set(GRB_CharAttr   attr, const GRBQConstr* xconstrs,
             const char*   val, int len);
    void set(GRB_DoubleAttr attr, const GRBQConstr* xconstrs,
             const double* val, int len);
    void set(GRB_StringAttr attr, const GRBQConstr* xconstrs,
             const string* val, int len);

    void setCallback(GRBCallback* xcb);
};

class GRBVarRep;

class GRBVar
{
  private:

    GRBVarRep* varRep;

  public:

    friend class GRBModel;
    friend class GRBLinExpr;
    friend class GRBQuadExpr;
    friend class GRBCallback;

    GRBVar();
    int get(GRB_IntAttr attr) const;
    char get(GRB_CharAttr attr) const;
    double get(GRB_DoubleAttr attr) const;
    string get(GRB_StringAttr attr) const;

    void set(GRB_IntAttr attr, int value);
    void set(GRB_CharAttr attr, char value);
    void set(GRB_DoubleAttr attr, double value);
    void set(GRB_StringAttr attr, const string& value);

    bool sameAs(GRBVar v2);
};

class GRBConRep;

class GRBConstr
{
  private:

    GRBConRep* conRep;

  public:

    friend class GRBModel;
    friend class GRBColumn;

    GRBConstr();
    int get(GRB_IntAttr attr) const;
    char get(GRB_CharAttr attr) const;
    double get(GRB_DoubleAttr attr) const;
    string get(GRB_StringAttr attr) const;

    void set(GRB_IntAttr attr, int value);
    void set(GRB_CharAttr attr, char value);
    void set(GRB_DoubleAttr attr, double value);
    void set(GRB_StringAttr attr, const string& value);

    bool sameAs(GRBConstr c2);
};

class GRBExpr 
{
  private:

  public:
    friend class GRBModel;    
};

class GRBLinExpr: public GRBExpr
{
  private:

    double constant;
    vector<double> coeffs;
    vector<GRBVar> vars;
  public:

    GRBLinExpr(double constant=0.0);
    GRBLinExpr(GRBVar var, double coeff=1.0);

    friend class GRBQuadExpr;

    friend ostream& operator<<(ostream &stream, GRBLinExpr expr);
    friend GRBLinExpr operator+(const GRBLinExpr& x, const GRBLinExpr& y);
    friend GRBLinExpr operator+(const GRBLinExpr& x);
    friend GRBLinExpr operator+(GRBVar x, GRBVar y);
    friend GRBLinExpr operator+(GRBVar x, double a);
    friend GRBLinExpr operator+(double a, GRBVar x);
    friend GRBLinExpr operator-(const GRBLinExpr& x, const GRBLinExpr& y);
    friend GRBLinExpr operator-(const GRBLinExpr& x);
    friend GRBLinExpr operator-(GRBVar x);
    friend GRBLinExpr operator-(GRBVar x, GRBVar y);
    friend GRBLinExpr operator-(GRBVar x, double a);
    friend GRBLinExpr operator-(double a, GRBVar x);
    friend GRBLinExpr operator*(double a, GRBVar x);
    friend GRBLinExpr operator*(GRBVar x, double a);
    friend GRBLinExpr operator*(const GRBLinExpr& x, double a);
    friend GRBLinExpr operator*(double a, const GRBLinExpr& x);
    friend GRBLinExpr operator/(GRBVar x, double a);
    friend GRBLinExpr operator/(const GRBLinExpr& x, double a);

    unsigned int size(void) const;
    GRBVar getVar(int i) const;
    double getCoeff(int i) const; 
    double getConstant() const;
    double getValue() const;

    void addTerms(const double* coeff, const GRBVar* var, int cnt);
    GRBLinExpr operator=(const GRBLinExpr& rhs);
    void operator+=(const GRBLinExpr& expr);
    void operator-=(const GRBLinExpr& expr);
    void operator*=(double mult);
    void operator/=(double a);
    GRBLinExpr operator+(const GRBLinExpr& rhs);
    GRBLinExpr operator-(const GRBLinExpr& rhs);
    void remove(int i);
    bool remove(GRBVar v);

    void clear();
};

class GRBQuadExpr: public GRBExpr
{
  private:

    GRBLinExpr linexpr;
    vector<double> coeffs;
    vector<GRBVar> vars1;
    vector<GRBVar> vars2;

  public:

    GRBQuadExpr(double constant=0.0);
    GRBQuadExpr(GRBVar var, double coeff=1.0);
    GRBQuadExpr(GRBLinExpr le);

    friend ostream& operator<<(ostream &stream, GRBQuadExpr expr);
    friend GRBQuadExpr operator+(const GRBQuadExpr& x, const GRBQuadExpr& y);
    friend GRBQuadExpr operator-(const GRBQuadExpr& x, const GRBQuadExpr& y);
    friend GRBQuadExpr operator+(const GRBQuadExpr& x);
    friend GRBQuadExpr operator-(const GRBQuadExpr& x);
    friend GRBQuadExpr operator*(const GRBQuadExpr& x, double a);
    friend GRBQuadExpr operator*(double a, const GRBQuadExpr& x);
    friend GRBQuadExpr operator*(GRBVar x, GRBVar y);
    friend GRBQuadExpr operator*(GRBVar x, const GRBLinExpr& y);
    friend GRBQuadExpr operator*(const GRBLinExpr& y, GRBVar x);
    friend GRBQuadExpr operator*(const GRBLinExpr& x, const GRBLinExpr& y);
    friend GRBQuadExpr operator/(const GRBQuadExpr& x, double a);

    unsigned int size(void) const;
    GRBVar getVar1(int i) const;
    GRBVar getVar2(int i) const;
    double getCoeff(int i) const;
    GRBLinExpr getLinExpr() const;
    double getValue() const;

    void addConstant(double c);
    void addTerm(double coeff, GRBVar var);
    void addTerm(double coeff, GRBVar var1, GRBVar var2);
    void addTerms(const double* coeff, const GRBVar* var, int cnt);
    void addTerms(const double* coeff, const GRBVar* var1, 
                  const GRBVar* var2, int cnt);
    void add(const GRBLinExpr le);
    GRBQuadExpr operator=(const GRBQuadExpr& rhs);
    void operator+=(const GRBQuadExpr& expr);
    void operator-=(const GRBQuadExpr& expr);
    void operator*=(double mult);
    void operator/=(double a);
    GRBQuadExpr operator+(const GRBQuadExpr& rhs);
    GRBQuadExpr operator-(const GRBQuadExpr& rhs);
    void remove(int i);
    bool remove(GRBVar v);

    void clear();
};

class GRBException
{
  private:

    string msg;
    int error;

  public:

    GRBException(int errcode = 0);
    GRBException(string errmsg, int errcode = 0); 
      
    const string getMessage() const;
    int getErrorCode() const;
};

class GRBCallback
{
  private:

    GRBmodel*   Cmodel;
    int         cols;
    void*       cbdata;
    double*     x;
    double*     newx;
    double*     relx;


  public:

    friend void GRBModel::setCallback(GRBCallback* cb);
    friend void GRBModel::update();

    GRBCallback();
    virtual ~GRBCallback() {};

  protected:

    int where;
    virtual void callback() {};
    double getDoubleInfo(int what);
    int getIntInfo(int what);
    const string getStringInfo(int what) const;
    double getSolution(GRBVar v);
    double* getSolution(const GRBVar* xvars, int len);
    double getNodeRel(GRBVar v);
    double* getNodeRel(const GRBVar* xvars, int len);
    void setSolution(GRBVar v, double val);
    void setSolution(const GRBVar* xvars, const double* sol, int len);
    void addCut(const GRBTempConstr& tc);
    void addCut(const GRBLinExpr& expr, char sense, double rhs);
    void addLazy(const GRBTempConstr& tc);
    void addLazy(const GRBLinExpr& expr, char sense, double rhs);
    void abort();
};

class GRBColumn
{
  private:

    vector<double> coeffs;
    vector<GRBConstr> constrs;

  public:

    unsigned int size(void) const;
    GRBConstr getConstr(int i) const;
    double getCoeff(int i) const; 

    void addTerm(double coeff, GRBConstr constr);
    void addTerms(const double* coeff, const GRBConstr* constr, int cnt);
    void remove(int i);
    bool remove(GRBConstr c);

    void clear();
};

class GRBSOSRep;

class GRBSOS
{
  private:

    GRBSOSRep* sosRep;
   
  public:

    friend class GRBModel;

    GRBSOS();
    int get(GRB_IntAttr attr) const;
};     

class GRBQConstrRep;

class GRBQConstr
{
  private:

    GRBQConstrRep* qconRep;
   
  public:

    friend class GRBModel;

    GRBQConstr();
    char get(GRB_CharAttr attr) const;
    double get(GRB_DoubleAttr attr) const;
    string get(GRB_StringAttr attr) const;

    void set(GRB_CharAttr attr, char value);
    void set(GRB_DoubleAttr attr, double value);
    void set(GRB_StringAttr attr, const string& value);
};     

class GRBTempConstr
{
  private:

    GRBQuadExpr expr;
    char        sense;

  public:

    friend class GRBModel;
    friend class GRBCallback;
    friend GRBTempConstr operator<=(GRBQuadExpr x, GRBQuadExpr y);
    friend GRBTempConstr operator>=(GRBQuadExpr x, GRBQuadExpr y);
    friend GRBTempConstr operator==(GRBQuadExpr x, GRBQuadExpr y);
};
#endif

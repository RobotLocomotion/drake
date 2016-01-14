#include "NonlinearProgram.h"

#include <string.h>

namespace snopt {
#include "snfilewrapper.hh"
}

#define INF 1.1e+20

using namespace std;

namespace drake {
const NonlinearProgram* thisNP;

NonlinearProgram::NonlinearProgram(snopt::integer num_vars) : m_num_vars(num_vars) {}

void NonlinearProgram::addConstraint(unique_ptr<Constraint>& cnstr) {
  m_constraints.push_back(move(cnstr));
}

void NonlinearProgram::setCost(unique_ptr<Constraint>& cost) {
  m_cost = move(cost);
}

void NonlinearProgram::generateA(snopt::integer* lenA,
    snopt::integer* neA,
    vector<snopt::integer>* iAfun,
    vector<snopt::integer>* jAvar,
    vector<snopt::doublereal>* A) const {
  *lenA = m_num_vars * (m_constraints.size() + 1);

  // Generate first row from cost's A
  snopt::integer costNeA;
  vector<snopt::integer> costJAvar;
  vector<snopt::doublereal> costA;
  m_cost->getA(&costNeA, &costJAvar, &costA);
  for (int i = 0; i < costNeA; ++i) {
    (*neA)++;
    iAfun->push_back(1);
    jAvar->push_back(costJAvar[i]);
    A->push_back(costA[i]);
  }

  // Generate remaining rows from constraints
  int num_constraints = m_constraints.size();
  for (int i = 0; i < num_constraints; ++i) {
    unique_ptr<Constraint> const& constr = m_constraints[i];
    snopt::integer thisNeA;
    vector<snopt::integer> thisJAvar;
    vector<snopt::doublereal> thisA;
    constr->getA(&thisNeA, &thisJAvar, &thisA);
    for (int j = 0; j < thisNeA; ++j) {
      (*neA)++;
      iAfun->push_back(i + 2);
      jAvar->push_back(thisJAvar[j]);
      A->push_back(thisA[j]);
    }
  }
}

void NonlinearProgram::generateG(snopt::integer* lenG,
    snopt::integer* neG,
    vector<snopt::integer>* iGfun,
    vector<snopt::integer>* jGvar) const {
  *lenG = m_num_vars * (m_constraints.size() + 1);

  // Generate first row from cost's G
  snopt::integer costNeG;
  vector<snopt::integer> costJGvar;
  m_cost->getG(&costNeG, &costJGvar);
  for (int i = 0; i < costNeG; ++i) {
    (*neG)++;
    iGfun->push_back(1);
    jGvar->push_back(costJGvar[i]);
  }

  // Generate remaining rows from constraints
  int num_constraints = m_constraints.size();
  for (int i = 0; i < num_constraints; ++i) {
    unique_ptr<Constraint> const& constr = m_constraints[i];
    snopt::integer thisNeG;
    vector<snopt::integer> thisJGvar;
    constr->getG(&thisNeG, &thisJGvar);
    for (int j = 0; j < thisNeG; ++j) {
      (*neG)++;
      iGfun->push_back(i + 2);
      jGvar->push_back(thisJGvar[j]);
    }
  }
}

int NonlinearProgram::userFun(snopt::integer *status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *nF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *lenG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru) {
  bool needF_ = (*needF) > 0;
  bool needG_ = (*needG) > 0;
  int iG = 0;

  // Calculate cost's portion of F and G
  snopt::doublereal f_cost;
  vector<snopt::doublereal> g_cost;
  thisNP->m_cost->nonlinearEval(x, needF_, needG_, &f_cost, &g_cost);
  if (needF_) {
    F[0] = f_cost;
  }
  if (needG_) {
    int sizeG = g_cost.size();
    for (int i = 0; i < sizeG; ++i) {
      G[iG] = g_cost[i];
      ++iG;
    }
  }

  // Calculate constraint portion of F and G
  int num_constraints = thisNP->m_constraints.size();
  for (int i = 0; i < num_constraints; ++i) {
    unique_ptr<Constraint> const& constr = thisNP->m_constraints[i];
    snopt::doublereal f_constr;
    vector<snopt::doublereal> g_constr;
    constr->nonlinearEval(x, needF_, needG_, &f_constr, &g_constr);
    if (needF_) {
      F[i + 1] = f_constr;
    }
    if (needG_) {
      int sizeG = g_constr.size();
      for (int j = 0; j < sizeG; ++j) {
        G[iG] = g_constr[j];
        ++iG;
      }
    }
  }
  return 0;
}

void NonlinearProgram::generateFBounds(vector<snopt::doublereal>* flow,
    vector<snopt::doublereal>* fupp) const {
  snopt::doublereal cost_low;
  snopt::doublereal cost_upp;
  m_cost->getBounds(&cost_low, &cost_upp);
  flow->push_back(cost_low);
  fupp->push_back(cost_upp);

  int num_constraints = m_constraints.size();
  for (int i = 0; i < num_constraints; ++i) {
    unique_ptr<Constraint> const& constr = m_constraints[i];
    snopt::doublereal constr_low;
    snopt::doublereal constr_upp;
    constr->getBounds(&constr_low, &constr_upp);
    flow->push_back(constr_low);
    fupp->push_back(constr_upp);
  }
}

void NonlinearProgram::solve(vector<snopt::doublereal>* x,
    snopt::doublereal* objval,
    snopt::integer* info) const {
  snopt::integer start = 0;
  snopt::integer n = m_num_vars;
  snopt::integer nf = 1 + m_constraints.size();
  snopt::integer nxname = 1;
  snopt::integer nfname = 1;
  snopt::doublereal objadd = 0;
  snopt::integer objrow = 1;
  char prob[] = "";
  snopt::integer mincw;
  snopt::integer miniw;
  snopt::integer minrw;
  snopt::integer ns;
  snopt::integer ninf;
  snopt::doublereal sinf;
  snopt::integer lencu = 0;
  snopt::integer leniu = 0;
  snopt::integer lenru = 0;
  vector<char> cu(lencu);
  vector<snopt::integer> iu(leniu);
  vector<snopt::doublereal> ru(lenru);
  snopt::integer lencw = 8*500;
  snopt::integer leniw = 10000;
  snopt::integer lenrw = 20000;
  vector<char> cw(lencw);
  vector<snopt::integer> iw(leniw);
  vector<snopt::doublereal> rw(lenrw);
  int name_len = strlen(prob);
  int xnames_len = 8*nxname;
  int fnames_len = 8*nfname;

  snopt::integer iPrint = 9;
  snopt::integer prnt_len;
  snopt::integer iSumm = 6;

  char printname[200];
  sprintf(printname, "%s", "snopt.out");
  prnt_len = strlen(printname);
  snopt::snopenappend_(&iPrint, printname, info, prnt_len);

  snopt::sninit_(&iPrint, &iSumm,
      cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw, lencw);

  // Generate A
  snopt::integer lena = 0;
  snopt::integer nea = 0;
  vector<snopt::integer> iafun;
  vector<snopt::integer> javar;
  vector<snopt::doublereal> a;
  generateA(&lena, &nea, &iafun, &javar, &a);
  printf("lena: %ld\nnea: %ld\n", lena, nea); 

  // Generate G
  snopt::integer leng = 0;
  snopt::integer neg = 0;
  vector<snopt::integer> igfun;
  vector<snopt::integer> jgvar;
  generateG(&leng, &neg, &igfun, &jgvar);

  // Get Bounds
  vector<snopt::doublereal> xlow;
  vector<snopt::doublereal> xupp;
  vector<char> xnames;
  vector<snopt::doublereal> xmul(n, 0.0);
  vector<snopt::integer> xstate(n, 0);
  for (int i = 0; i < n; ++i) {
    xlow.push_back(-INF);
    xupp.push_back(INF);
  }
  vector<snopt::doublereal> flow;
  vector<snopt::doublereal> fupp;
  vector<char> fnames;
  vector<snopt::doublereal> f(nf);
  vector<snopt::doublereal> fmul(nf, 0.0);
  vector<snopt::integer> fstate(nf, 0);
  generateFBounds(&flow, &fupp);

  // HACKY
  thisNP = this;

  debugPrint(n, nf, lena, nea, iafun, javar, a, leng, neg, igfun, jgvar, xlow, xupp, flow, fupp);

  // Solve
  snopt::snopta_(&start, &nf, &n,
    &nxname, &nfname, &objadd, &objrow,
    prob, userFun,
    iafun.data(), javar.data(), &lena, &nea, a.data(),
    igfun.data(), jgvar.data(), &leng, &neg,
    xlow.data(), xupp.data(), xnames.data(),
    flow.data(), fupp.data(), fnames.data(),
    x->data(), xstate.data(), xmul.data(),
    f.data(), fstate.data(), fmul.data(),
    info, &mincw, &miniw, &minrw, &ns, &ninf, &sinf,
    cu.data(), &lencu, iu.data(), &leniu, ru.data(), &lenru,
    cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw,
    name_len, xnames_len, fnames_len, lencu, lencw);
}

void NonlinearProgram::debugPrint(snopt::integer const& n,
    snopt::integer const& nf,
    snopt::integer const& lena,
    snopt::integer const& nea,
    vector<snopt::integer> const& iafun,
    vector<snopt::integer> const& javar,
    vector<snopt::doublereal> const& a,
    snopt::integer const& leng,
    snopt::integer const& neg,
    vector<snopt::integer> const& igfun,
    vector<snopt::integer> const& jgvar,
    vector<snopt::doublereal> const& xlow,
    vector<snopt::doublereal> const& xupp,
    vector<snopt::doublereal> const& flow,
    vector<snopt::doublereal> const& fupp) const {
  printf("==========\n%s\n", "DEBUG INFORMATION");

  printf("N: %ld\nNF: %ld\n", n, nf);
  printf("len(A): %ld\nneA: %ld\nlen(iafun): %ld\nlen(javar): %ld\n", lena, nea, iafun.size(), javar.size());
  for (int i = 0; i < iafun.size(); ++i) {
    printf("\tA(%ld,%ld): %e\n", iafun[i], javar[i], a[i]);
  }
  printf("len(G): %ld\nneG: %ld\nlen(igfun): %ld\nlen(jgvar): %ld\n", leng, neg, igfun.size(), jgvar.size());
  for (int i = 0; i < igfun.size(); ++i) {
    printf("\tG(%ld,%ld)\n", igfun[i], jgvar[i]);
  }
  printf("len(xlow): %ld\nlen(xupp): %ld\n", xlow.size(), xupp.size());
  for (int i = 0; i < xlow.size(); ++i) {
    printf("\txlow(%d): %e\txupp(%d): %e\n", i, xlow[i], i, xupp[i]);
  }
  printf("len(flow): %ld\nlen(fupp): %ld\n", flow.size(), fupp.size());
  for (int i = 0; i < flow.size(); ++i) {
    printf("\tflow(%d): %e\tfupp(%d): %e\n", i, flow[i], i, fupp[i]);
  }

  printf("%s\n", "==========");
}
} // namespace drake

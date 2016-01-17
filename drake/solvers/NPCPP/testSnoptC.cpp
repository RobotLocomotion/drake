#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>

namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
}

#define INF 1.1e+20

using namespace std;

int userfunSampleProblem(snopt::integer *status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *nF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *lenG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru) {
  if (*needF > 0) {
    F[0] = x[1];
    F[1] = x[0]*x[0] + 4*x[1]*x[1];
    F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
  }
  if (*needG > 0) {
    G[0] = 2*x[0];
    G[1] = 8*x[1];
    G[2] = 2*(x[0] - 2);
    G[3] = 2*x[1];
  }
  return 0;
}

void solveSampleProblem() {
  /**
   * Solves:
   * min    x2
   * st     x1^2 + 4*x2^2 <= 4
   *        (x1 - 2)^2 + x2^2 <= 5
   *        x1 >= 0
   */
  snopt::integer start = 0;
  snopt::integer nf = 3;
  snopt::integer n = 2;
  snopt::integer nxname = 1;
  snopt::integer nfname = 1;
  snopt::doublereal objadd = 0.0;
  snopt::integer objrow = 1;
  char prob[] = "Sample";

  snopt::integer lena = 6;
  vector<snopt::integer> iafun(lena);
  vector<snopt::integer> javar(lena);
  vector<snopt::doublereal> a(lena);
  iafun[0] = 1; javar[0] = 2; a[0] = 1.0;
  snopt::integer nea = 1;

  snopt::integer leng = 6;
  vector<snopt::integer> igfun(leng);
  vector<snopt::integer> jgvar(leng);
  igfun[0] = 2; jgvar[0] = 1;
  igfun[1] = 2; jgvar[1] = 2;
  igfun[2] = 3; jgvar[2] = 1;
  igfun[3] = 3; jgvar[3] = 2;
  snopt::integer neg = 4;

  vector<snopt::doublereal> xlow(n);
  vector<snopt::doublereal> xupp(n);
  vector<char> xnames;
  vector<snopt::doublereal> x(n, 1.0);
  vector<snopt::doublereal> xmul(n, 0.0);
  vector<snopt::integer> xstate(n, 0);
  xlow[0] = 0; xlow[1] = -INF;
  xupp[0] = INF; xupp[1] = INF;

  vector<snopt::doublereal> flow(nf);
  vector<snopt::doublereal> fupp(nf);
  vector<char> fnames;
  vector<snopt::doublereal> f(nf);
  vector<snopt::doublereal> fmul(nf, 0.0);
  vector<snopt::integer> fstate(nf, 0);
  flow[0] = -INF; flow[1] = -INF; flow[2] = -INF;
  fupp[0] = INF; fupp[1] = 4.0; fupp[2] = 5.0;

  snopt::integer info;
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
  int name_len = 6;
  int xnames_len = 8*nxname;
  int fnames_len = 8*nfname;

  snopt::integer iPrint = 9;
  snopt::integer prnt_len;
  snopt::integer iSumm = 6;

  char printname[200];
  sprintf(printname, "%s", "sample.out");
  prnt_len = strlen(printname);
  snopt::snopenappend_(&iPrint, printname, &info, prnt_len);

  snopt::sninit_(&iPrint, &iSumm,
      cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw, lencw);
  
  snopt::snopta_(&start, &nf, &n,
      &nxname, &nfname, &objadd, &objrow,
      prob, userfunSampleProblem,
      iafun.data(), javar.data(), &lena, &nea, a.data(),
      igfun.data(), jgvar.data(), &leng, &neg,
      xlow.data(), xupp.data(), xnames.data(),
      flow.data(), fupp.data(), fnames.data(),
      x.data(), xstate.data(), xmul.data(),
      f.data(), fstate.data(), fmul.data(),
      &info, &mincw, &miniw, &minrw, &ns, &ninf, &sinf,
      cu.data(), &lencu, iu.data(), &leniu, ru.data(), &lenru,
      cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw,
      name_len, xnames_len, fnames_len, lencu, lencw);

}

int userfunSampleProblem2(snopt::integer *status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *nF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *lenG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru) {
  if (*needF > 0) {
    F[0] = 0;
    F[1] = x[0]*x[0] - 6*x[0];
  }
  if (*needG > 0) {
    G[0] = 2*x[0] - 6;
  }
  return 0;
}

void solveSampleProblem2() {
  /**
   * Solves:
   * min    x2
   * st     x1^2 - 6*x1 - y = -9
   *
   * Otherwise known as minimum of y = (x - 3)^2
   */
  snopt::integer start = 0;
  snopt::integer nf = 2;
  snopt::integer n = 2;
  snopt::integer nxname = 1;
  snopt::integer nfname = 1;
  snopt::doublereal objadd = 0.0;
  snopt::integer objrow = 1;
  char prob[] = "Sample2";

  snopt::integer lena = 4;
  vector<snopt::integer> iafun(lena);
  vector<snopt::integer> javar(lena);
  vector<snopt::doublereal> a(lena);
  iafun[0] = 1; javar[0] = 2; a[0] = 1.0;
  iafun[1] = 2; javar[1] = 2; a[1] = -1.0;
  snopt::integer nea = 2;

  snopt::integer leng = 4;
  vector<snopt::integer> igfun(leng);
  vector<snopt::integer> jgvar(leng);
  igfun[0] = 2; jgvar[0] = 1;
  snopt::integer neg = 1;

  vector<snopt::doublereal> xlow(n);
  vector<snopt::doublereal> xupp(n);
  vector<char> xnames;
  vector<snopt::doublereal> x(n, 1.0);
  vector<snopt::doublereal> xmul(n, 0.0);
  vector<snopt::integer> xstate(n, 0);
  xlow[0] = -INF; xlow[1] = -INF;
  xupp[0] = INF; xupp[1] = INF;

  vector<snopt::doublereal> flow(nf);
  vector<snopt::doublereal> fupp(nf);
  vector<char> fnames;
  vector<snopt::doublereal> f(nf);
  vector<snopt::doublereal> fmul(nf, 0.0);
  vector<snopt::integer> fstate(nf, 0);
  flow[0] = -INF; flow[1] = -9;
  fupp[0] = INF; fupp[1] = -9;

  snopt::integer info;
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
  sprintf(printname, "%s", "sample2.out");
  prnt_len = strlen(printname);
  snopt::snopenappend_(&iPrint, printname, &info, prnt_len);

  snopt::sninit_(&iPrint, &iSumm,
      cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw, lencw);
  
  snopt::snopta_(&start, &nf, &n,
      &nxname, &nfname, &objadd, &objrow,
      prob, userfunSampleProblem2,
      iafun.data(), javar.data(), &lena, &nea, a.data(),
      igfun.data(), jgvar.data(), &leng, &neg,
      xlow.data(), xupp.data(), xnames.data(),
      flow.data(), fupp.data(), fnames.data(),
      x.data(), xstate.data(), xmul.data(),
      f.data(), fstate.data(), fmul.data(),
      &info, &mincw, &miniw, &minrw, &ns, &ninf, &sinf,
      cu.data(), &lencu, iu.data(), &leniu, ru.data(), &lenru,
      cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw,
      name_len, xnames_len, fnames_len, lencu, lencw);

}


int userfunSampleProblem3(snopt::integer *status, snopt::integer *n, snopt::doublereal x[],
    snopt::integer *needF, snopt::integer *nF, snopt::doublereal F[],
    snopt::integer *needG, snopt::integer *lenG, snopt::doublereal G[],
    char *cu, snopt::integer *lencu,
    snopt::integer iu[], snopt::integer *leniu,
    snopt::doublereal ru[], snopt::integer *lenru) {
  if (*needF > 0) {
    F[0] = 0;
    F[1] = 0;
  }
  if (*needG > 0) {
  }
  return 0;
}

void solveSampleProblem3() {
  /**
   * Solves:
   * min    x2
   * st     x1 + x2 = 10
   *        1 <= x1 <= 9
   *
   * Otherwise known as minimum of y = -x + 10
   */
  snopt::integer start = 0;
  snopt::integer nf = 2;
  snopt::integer n = 2;
  snopt::integer nxname = 1;
  snopt::integer nfname = 1;
  snopt::doublereal objadd = 0.0;
  snopt::integer objrow = 1;
  char prob[] = "Sample2";

  snopt::integer lena = 4;
  vector<snopt::integer> iafun(lena);
  vector<snopt::integer> javar(lena);
  vector<snopt::doublereal> a(lena);
  iafun[0] = 1; javar[0] = 2; a[0] = 1.0;
  iafun[1] = 2; javar[1] = 1; a[1] = 1.0;
  iafun[2] = 2; javar[2] = 2; a[2] = 1.0;
  snopt::integer nea = 3;

  snopt::integer leng = 0;
  vector<snopt::integer> igfun(leng);
  vector<snopt::integer> jgvar(leng);
  snopt::integer neg = 0;

  vector<snopt::doublereal> xlow(n);
  vector<snopt::doublereal> xupp(n);
  vector<char> xnames;
  vector<snopt::doublereal> x(n, 1.0);
  vector<snopt::doublereal> xmul(n, 0.0);
  vector<snopt::integer> xstate(n, 0);
  xlow[0] = 1; xlow[1] = -INF;
  xupp[0] = 9; xupp[1] = INF;

  vector<snopt::doublereal> flow(nf);
  vector<snopt::doublereal> fupp(nf);
  vector<char> fnames;
  vector<snopt::doublereal> f(nf);
  vector<snopt::doublereal> fmul(nf, 0.0);
  vector<snopt::integer> fstate(nf, 0);
  flow[0] = -INF; flow[1] = 10.0;
  fupp[0] = INF; fupp[1] = 10.0;

  snopt::integer info;
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
  sprintf(printname, "%s", "sample3.out");
  prnt_len = strlen(printname);
  snopt::snopenappend_(&iPrint, printname, &info, prnt_len);

  snopt::sninit_(&iPrint, &iSumm,
      cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw, lencw);
  
  snopt::snopta_(&start, &nf, &n,
      &nxname, &nfname, &objadd, &objrow,
      prob, userfunSampleProblem3,
      iafun.data(), javar.data(), &lena, &nea, a.data(),
      igfun.data(), jgvar.data(), &leng, &neg,
      xlow.data(), xupp.data(), xnames.data(),
      flow.data(), fupp.data(), fnames.data(),
      x.data(), xstate.data(), xmul.data(),
      f.data(), fstate.data(), fmul.data(),
      &info, &mincw, &miniw, &minrw, &ns, &ninf, &sinf,
      cu.data(), &lencu, iu.data(), &leniu, ru.data(), &lenru,
      cw.data(), &lencw, iw.data(), &leniw, rw.data(), &lenrw,
      name_len, xnames_len, fnames_len, lencu, lencw);

}

int main() {
  solveSampleProblem();

  solveSampleProblem2();

  solveSampleProblem3();

  return 0;
}

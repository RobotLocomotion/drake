#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

#include "sparseQP.h"

typedef struct probInfo {
  char fileName[512];           /* data file name */
  int mode;                     /* 3==binary, 2=ascii */
  int m;                        /* row count */
  int n;                        /* column count */
  int nnz;                      /* constraint nonzeros */
  /* L is the lower triangular portion of the symmetric Q */
  int Lnz;                      /* nnz in L */
  int Qnz;                      /* nnz in Q */
} probInfo_t;

#define GETMEM(PTR,TYPE,N) (PTR)=(TYPE*)malloc((N)*sizeof(TYPE)); assert(PTR)
#define FREEMEM(PTR) do {if ((PTR)) { free(PTR);(PTR) = NULL;}} while (0)

static int mustReverse = 0;

static void
getInfo (probInfo_t *pi, const char *fName)
{
  FILE *fp;
  size_t sz;
  char *s;
  char buf[512];
  int v;

  (void) memset (pi, 0, sizeof(probInfo_t));
  pi->m = -1;
  pi->n = -1;
  pi->nnz = -1;
  pi->Lnz = -1;

  if (fName) {
    if ((fp = fopen(fName, "r")) == NULL) {
      perror ("fopen");
      printf ("meta-data file '%s' could not be opened\n", fName);
      assert (fp);
    }
    while (! feof(fp)) {
      s = fgets (buf, sizeof(buf)-1, fp);
      if (!s)
        break;
      buf[sizeof(buf)-1] = '\0';
      sz = strlen(buf);
      s = buf + sz - 1;
      if ('\n' == *s)
        *s-- = '\0';
      while ((s >= buf) && (' ' == *s))
        *s-- = '\0';
      printf ("read line: '%s'\n", buf);
      if ('*' == buf[0])
        continue;
      if (0==strncmp("n ", buf, 2)) {
        sscanf (buf+2, "%d", &v);
        printf ("  N: %d\n", v);
        pi->n = v;
      }
      if (0==strncmp("m ", buf, 2)) {
        sscanf (buf+2, "%d", &v);
        printf ("  M: %d\n", v);
        pi->m = v;
      }
      if (0==strncmp("nnz ", buf, 4)) {
        sscanf (buf+4, "%d", &v);
        printf ("  NNZ: %d\n", v);
        pi->nnz = v;
      }
      if (0==strncmp("Lnz ", buf, 4)) {
        sscanf (buf+4, "%d", &v);
        printf ("  LNZ: %d\n", v);
        pi->Lnz = v;
      }
      if (0==strncmp("mode ", buf, 5)) {
        s = buf + 5;
        while (' ' == *s)
          s++;
        printf ("  MODE: %s\n", s);
        if (0 == strcmp("binary",s))
          pi->mode = 3;
        else if (0 == strcmp("ascii",s))
          pi->mode = 2;
        else
          printf ("  Unknown file mode '%s'\n", s);
      }
      if (0==strncmp("file ", buf, 5)) {
        s = buf + 5;
        while (' ' == *s)
          s++;
        printf ("  FILE: %s\n", s);
        strcpy (pi->fileName, s);
      }
    }
    fclose(fp);
    fp = NULL;
    assert(pi->m >= 0);
    assert(pi->n > 0);
    assert(pi->nnz >= 0);
    assert(pi->Lnz >= 0);
  }
  else {
    pi->m = 3;
    pi->n = 2;
    pi->nnz = 6;
    pi->Lnz = 3;
  }
} /* getInfo */

static void
reverseBytes (void *s, int nItems, int nBytes)
{
  int i;
  unsigned char tmp[8];
  unsigned char *f, *t;
  unsigned char *src = s;

  for (i = 0;  i < nItems;  i++) {
    memcpy (tmp, src, nBytes);
    for (f = tmp+nBytes, t=src;  f > tmp;  ) {
      *t++ = *--f;
    }
    src += nBytes;
  }
} /* reverseBytes */

static int
readCVec (FILE *fp, int mode, char v[], int dim)
{
  int i, k;
  int t;

  if (3 == mode) {   /* binary */
    for (i = 0;  i < dim;  i++) {
      k = fread (v+i, sizeof(char), 1, fp);
      if (k!=1) return 1;
    }
  }
  else {
    for (i = 0;  i < dim;  i++) {
      k = fscanf (fp, "%d", &t);
      if (k!=1) return 1;
      v[i] = (char) t;
    }
  }
  return 0;
} /* readCVec */

static int
readIVec (FILE *fp, int mode, int v[], int dim)
{
  int i, k;

  if (3 == mode) {   /* binary */
    for (i = 0;  i < dim;  i++) {
      k = fread (v+i, sizeof(int), 1, fp);
      if (k!=1) return 1;
    }
    if (mustReverse) {
      reverseBytes (v, dim, sizeof(int));
    }
  }
  else {
    for (i = 0;  i < dim;  i++) {
      k = fscanf (fp, "%d", v+i);
      if (k!=1) return 1;
    }
  }
  return 0;
} /* readIVec */

static int
readRVec (FILE *fp, int mode, double v[], int dim)
{
  int i, k;

  if (3 == mode) {   /* binary */
    for (i = 0;  i < dim;  i++) {
      k = fread (v+i, sizeof(double), 1, fp);
      if (k!=1) return 1;
    }
    if (mustReverse) {
      reverseBytes (v, dim, sizeof(double));
    }
  }
  else {
    for (i = 0;  i < dim;  i++) {
      k = fscanf (fp, "%lf", v+i);
      if (k!=1) return 1;
    }
  }
  return 0;
} /* readRVec */

static int
runningBigEndian (void)
{
  unsigned char cbuf[4] = "\x01\x00\x00\x00";
  unsigned int *pcbuf = (unsigned int *) cbuf;
  if (1 == *pcbuf) return 0;
  if (1*256*256*256 == *pcbuf) return 1;
  return 2; /* neither big nor little??? */
} /* runningBigEndian */

int
main (int argc, char **argv)
{
  double obj;
  QP_Termination termination;
  int i, j;
  probInfo_t pInfo;
  FILE *fpDat = NULL;
  char    *srowtypes;
  int     *iRowType;            /* int version of row types */
  double  *drhs;
  double  *dobj;
  int     *colPtr;
  int     *rowInd;
  double  *jacVal;
  double  *xlb;
  double  *xub;
  int     *Lcol1;
  int     *Lcol2;
  double  *Le;
  double  *x;                   /* primal solution */
  double  *pi;                  /* dual solution */
  int dataCreatedBigEndian = 0;
  int rc;

  mustReverse = (dataCreatedBigEndian != runningBigEndian ());
  if (argc <=1) {
    printf ("No filename arg: solving a tiny 'baked-in' QP model\n\n");
    getInfo (&pInfo, NULL);
  }
  else {
    assert(argc >= 2);
    getInfo (&pInfo, argv[1]);
    assert(pInfo.mode);
    assert(pInfo.fileName);
    if (2==pInfo.mode)
      fpDat = fopen (pInfo.fileName, "r");
    else
      fpDat = fopen (pInfo.fileName, "rb");
    if (NULL == fpDat) {
      perror ("fopen");
      printf ("data file '%s' could not be opened\n", pInfo.fileName);
      return 1;
    }
    assert(fpDat);
  }

  GETMEM(srowtypes, char,   pInfo.m);
  GETMEM(iRowType,  int,    pInfo.m);
  GETMEM(drhs,      double, pInfo.m);
  GETMEM(dobj,      double, pInfo.n);
  GETMEM(colPtr,    int,    pInfo.n+1);
  GETMEM(rowInd,    int,    pInfo.nnz);
  GETMEM(jacVal,    double, pInfo.nnz);
  GETMEM(xlb,       double, pInfo.n);
  GETMEM(xub,       double, pInfo.n);
  GETMEM(Lcol1,     int,    pInfo.Lnz);
  GETMEM(Lcol2,     int,    pInfo.Lnz);
  GETMEM(Le,        double, pInfo.Lnz);
  GETMEM(x,         double, pInfo.n);
  GETMEM(pi,        double, pInfo.m);

  if (fpDat) {
    rc = readCVec (fpDat, pInfo.mode, srowtypes, pInfo.m); assert(0==rc);
    rc = readRVec (fpDat, pInfo.mode, drhs,      pInfo.m); assert(0==rc);
    rc = readRVec (fpDat, pInfo.mode, dobj,      pInfo.n); assert(0==rc);
    rc = readIVec (fpDat, pInfo.mode, colPtr,    pInfo.n+1); assert(0==rc);
    rc = readIVec (fpDat, pInfo.mode, rowInd,    pInfo.nnz); assert(0==rc);
    rc = readRVec (fpDat, pInfo.mode, jacVal,    pInfo.nnz); assert(0==rc);
    rc = readRVec (fpDat, pInfo.mode, xlb,       pInfo.n); assert(0==rc);
    rc = readRVec (fpDat, pInfo.mode, xub,       pInfo.n); assert(0==rc);
    rc = readIVec (fpDat, pInfo.mode, Lcol1,     pInfo.Lnz); assert(0==rc);
    rc = readIVec (fpDat, pInfo.mode, Lcol2,     pInfo.Lnz); assert(0==rc);
    rc = readRVec (fpDat, pInfo.mode, Le,        pInfo.Lnz); assert(0==rc);
  }
  else {
    srowtypes[0] = 'G';
    srowtypes[1] = 'G';
    srowtypes[2] = 'G';
    drhs[0] = -8.0;
    drhs[1] = -5.5;
    drhs[2] = -8.0;
    dobj[0] = -6.0;
    dobj[1] = -12.0;
    colPtr[0] = 0;
    colPtr[1] = 3;
    colPtr[2] = 6;
    rowInd[0] = 0;
    rowInd[1] = 1;
    rowInd[2] = 2;
    rowInd[3] = 0;
    rowInd[4] = 1;
    rowInd[5] = 2;
    jacVal[0] = -4.0;
    jacVal[1] = -2.0;
    jacVal[2] = -1.0;
    jacVal[3] = -1.0;
    jacVal[4] = -2.0;
    jacVal[5] = -4.0;
    xlb[0] = 0.0;
    xlb[1] = 0.0;
    xub[0] = 1.0e+20;
    xub[1] = 1.0e+20;
    Lcol1[0] = 0;
    Lcol1[1] = 0;
    Lcol1[2] = 1;
    Lcol2[0] = 0;
    Lcol2[1] = 1;
    Lcol2[2] = 1;
    Le[0] = 2.0;
    Le[1] = 1.0;
    Le[2] = 2.0;
  }

  if (fpDat)
    (void) fclose(fpDat);
  for (i = 0;  i < pInfo.m;  i++) {
    switch (srowtypes[i]) {
    case 'G':
    case 'g':
      iRowType[i] = QP_GE;
      break;
    case 'L':
    case 'l':
      iRowType[i] = QP_LE;
      break;
    case 'E':
    case 'e':
      iRowType[i] = QP_EQ;
      break;
    default:
      assert('?'==srowtypes[i]);
    }
  }
  (void) memset (x , 0, sizeof(double)*pInfo.n);
  (void) memset (pi, 0, sizeof(double)*pInfo.m);
  sparseQP (pInfo.n, pInfo.m, pInfo.Lnz, Lcol2, Lcol1, Le, dobj,
            pInfo.nnz, colPtr, rowInd, jacVal, drhs, iRowType, xlb, xub,
            &termination, x, pi, &obj);

  switch (termination) {
  case QP_Solved:
    printf ("QP solved.\n");
    printf ("objective: %g\n", obj);
    if ((pInfo.n <= 10) && (pInfo.m <= 10)) {
      for (j = 0; j < pInfo.n; j++) {
        printf ("x[%3d] = %g\n", j+1, x[j]);
      }
      for (i = 0; i < pInfo.m; i++) {
        printf("pi[%3d] = %g\n", i+1, pi[i]);
      }
    }
    break;
  case QP_Unbounded:
    printf("QP unbounded.\n");
    break;
  case QP_Infeasible:
    printf("QP infeasible.\n");
    break;
  case QP_Error:
    printf("*** Error solving QP.\n");
    break;
  default:
    printf ("Unexpected termination status: %d\n", termination);
  }

  FREEMEM(srowtypes);
  FREEMEM(drhs);
  FREEMEM(dobj);
  FREEMEM(colPtr);
  FREEMEM(rowInd);
  FREEMEM(jacVal);
  FREEMEM(xlb);
  FREEMEM(xub);
  FREEMEM(Lcol1);
  FREEMEM(Lcol2);
  FREEMEM(Le);
  FREEMEM(x);
  FREEMEM(pi);

  return 0;
}

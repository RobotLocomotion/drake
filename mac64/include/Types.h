
/*****************************************************************************/
/* Types.h                                                                   */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Declarations for the types used throughout the interface.               */
/*****************************************************************************/

#ifndef TYPES_H
#define TYPES_H

#include "Config.h"
#include "Import.h"

/*****************************************************************************/
/* Data types.                                                               */
/*****************************************************************************/

#define Void     void
#define Char     char
#define Unsigned unsigned
#define Int      int
#define Long     long
#define Float    float
#define Double   double

#define Boolean  int
#define False    0
#define True     1

/*****************************************************************************/
/* Basis_Type - valid types when for the basis functions.                    */
/*****************************************************************************/

typedef Int Basis_Type;
#define Basis_Unknown    0      /* Basis type not known                      */
#define Basis_Basic      1      /* Strictly between bounds, variable basic   */
#define Basis_SuperBasic 2      /* Strictly between bounds, variable nonbasic*/
#define Basis_LowerBound 3      /* At lower bound, slack is basic            */
#define Basis_UpperBound 4      /* At upper bound, slack is basic            */
#define Basis_Fixed      5      /* Variable was fixed and removed            */

/*****************************************************************************/
/* MCP_Type - valid types for the MCP_SetProblemClass( ) function.           */
/*****************************************************************************/

typedef enum
{
  MCP_MCP,                  /* The problem is a general nonlinear mcp        */
  MCP_LCP,                  /* The problem is a linear mcp                   */
  MCP_NLP,                  /* The problem is KKT conditions for nlp         */
  MCP_QP,                   /* The problem is KKT conditions for qp          */
  MCP_LP,                   /* The problem is KKT conditions for lp          */
  MCP_CNS,                  /* The problem is a CNS reformulated as mcp      */
  MCP_MPSGE                 /* The problem is a general equilibrium          */
                            /* model defined using MPSGE syntax              */
} MCP_Type;

/*****************************************************************************/
/* MCP_Termination - valid return codes for Path_Solve( );                   */
/*****************************************************************************/

typedef enum
{
  MCP_Solved = 1,            /* The problem was solved                       */
  MCP_NoProgress,            /* A stationary point was found                 */
  MCP_MajorIterationLimit,   /* Major iteration limit met                    */
  MCP_MinorIterationLimit,   /* Cumulative minor iterlim met                 */
  MCP_TimeLimit,             /* Ran out of time                              */
  MCP_UserInterrupt,         /* Control-C, typically                         */
  MCP_BoundError,            /* Problem has a bound error                    */
  MCP_DomainError,           /* Could not find starting point                */
  MCP_Infeasible,            /* Problem has no solution                      */
  MCP_Error,                 /* An error occurred within the code            */
  MCP_Reduced,               /* Presolve reduced problem                     */
  MCP_LicenseError,          /* License could not be found                   */
  MCP_OK                     /* Presolve did not perform any modifications   */
} MCP_Termination;

/*****************************************************************************/
/* CNS_Type - valid types for the CNS_SetProblemClass( ) function.           */
/*****************************************************************************/

typedef enum
{
  CNS_CNS,                  /* The problem is a general nonlinear cns        */
  CNS_LCNS                  /* The problem is a linear cns                   */
} CNS_Type;

/*****************************************************************************/
/* CNS_Termination - valid return codes for CNS solvers                      */
/*****************************************************************************/

typedef enum
{
  CNS_Solved = 1,            /* The problem was solved                       */
  CNS_NoProgress,            /* A stationary point was found                 */
  CNS_MajorIterationLimit,   /* Major iteration limit met                    */
  CNS_MinorIterationLimit,   /* Cumulative minor iterlim met                 */
  CNS_TimeLimit,             /* Ran out of time                              */
  CNS_UserInterrupt,         /* Control-C, typically                         */
  CNS_BoundError,            /* Problem has a bound error                    */
  CNS_DomainError,           /* Could not find starting point                */
  CNS_Infeasible,            /* Problem has no solution                      */
  CNS_Error,                 /* An error occurred within the code            */
  CNS_Reduced,               /* Presolve reduced problem                     */
  CNS_LicenseError,          /* License could not be found                   */
  CNS_OK
} CNS_Termination;

/*****************************************************************************/
/* LCP_Type - valid types for the LCP_SetProblemClass( ) function.           */
/*****************************************************************************/

typedef enum
{
  LCP_LCP,                   /* The problem is a linear mcp                  */
  LCP_QP,                    /* The problem is KKT conditions for qp         */
  LCP_LP                     /* The problem is KKT conditions for lp         */
} LCP_Type;

/*****************************************************************************/
/* LCP_Termination - valid return codes for Lemke_Solve( );                  */
/*****************************************************************************/

typedef enum
{
  LCP_Solved = 1,            /* The problem was solved                       */
  LCP_Distance,              /* Distance limit has been reached              */
  LCP_RayTerm,               /* A ray termination occurred                   */
  LCP_Singular,              /* A singular basis was encountered             */
  LCP_Cycle,                 /* A cycle was detected                         */
  LCP_Reset,                 /* A reset was required, but not performed      */
  LCP_IterationLimit,        /* The iteration limit was reached              */
  LCP_TimeLimit,             /* The time limit was reached                   */
  LCP_Infeasible,            /* Problem has no solution                      */
  LCP_Error,                 /* An error occurred within the code            */
  LCP_Reduced,               /* Presolve reduced problem                     */
  LCP_OK
} LCP_Termination;

/*****************************************************************************/
/* License_Termination - valid return codes during license checks            */
/*****************************************************************************/

typedef enum
{
  LIC_VALID = 0,          /* License is valid */
  LIC_NOT_CHECKED,        /* License has not been checked yet */
  LIC_NOT_FOUND,          /* License string was not set */
  LIC_NOT_SUPPORTED,      /* License version is not supported */
  LIC_PARSE,              /* Failure to parse license string */
  LIC_SIGNATURE,          /* Signature does not match key */
  LIC_MAGIC,              /* Magic number for VAR license incorrect */
  LIC_ALGORITHM,          /* Algorithm selected not licensed */
  LIC_ARCHITECTURE,       /* Architecture not licensed */
  LIC_EXPIRATION,         /* License has expired */
  LIC_BUILD,              /* Library build not licensed */
  LIC_VERSION,            /* Library version not licensed */
  LIC_SIZE                /* Size limitation exceeded */
} License_Termination;

/*****************************************************************************/
/* Information - information passed into and out of the solver.              */
/*   The generate_output, use_start, and use_basics fields should be set on  */
/*   input.  The remainder are filled by the algorithm.                      */
/*****************************************************************************/

typedef struct
{
  Double residual;           /* Value of residual at final point             */
  Double distance;           /* Distance between initial and final point     */
  Double steplength;         /* Steplength taken                             */
  Double total_time;         /* Amount of time spent in the code             */
  Double basis_time;         /* Amount of time spent factoring               */

  Double maximum_distance;   /* Maximum distance from init point allowed     */

  Int major_iterations;      /* Major iterations taken                       */
  Int minor_iterations;      /* Minor iterations taken                       */
  Int crash_iterations;      /* Crash iterations taken                       */
  Int function_evaluations;  /* Function evaluations performed               */
  Int jacobian_evaluations;  /* Jacobian evaluations performed               */
  Int gradient_steps;        /* Gradient steps taken                         */
  Int restarts;              /* Restarts used                                */

  Int generate_output;       /* Mask where output can be displayed.          */
  Int generated_output;      /* Mask where output displayed.                 */

  Boolean forward;           /* Move forward?                                */
  Boolean backtrace;         /* Back track?                                  */
  Boolean gradient;          /* Take gradient step?                          */

  Boolean use_start;         /* Use the starting point provided?             */
  Boolean use_basics;        /* Use the basis provided?                      */

  Boolean used_start;        /* Was the starting point given used?           */
  Boolean used_basics;       /* Was the initial basis given used?            */
} Information;

/*****************************************************************************/
/* Output definitions - flags for the log, status, and listing modes.        */
/*****************************************************************************/

#define Output_Log     (1 << 0)
#define Output_Status  (1 << 1)
#define Output_Listing (1 << 2)

#endif

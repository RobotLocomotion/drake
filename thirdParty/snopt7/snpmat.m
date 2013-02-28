function [x,fval,exitflag,output,lambda] = snpmat(myobj,x0,A,b,varargin)

% A wrapper for snopt to make it look like fmincon.
%   [x,fval,exitflag,output,lambda] = snpmat(myobj,x0,A,b)
%   [x,fval,exitflag,output,lambda] = snpmat(myobj,x0,A,b,Aeq,beq)
%   [x,fval,exitflag,output,lambda] = snpmat(myobj,x0,A,b,Aeq,beq,xlow,xupp)
%   [x,fval,exitflag,output,lambda] = snpmat(myobj,x0,A,b,Aeq,beq,xlow,xupp,nonlcon,options)
%
% snpmat and fmincon assume problems are of the form:
%    minimize    f(x)
%   such that    c(x)   <= 0,
%                c_eq(x) = 0,
%                Ax     <= b,
%                A_eq x  = b_eq,
%                xlow   <= x <= xupp.
%
% 26 November 2012.

[mi,n] =  size(A);

if     nargin == 4,
  me       = 0;
  nli      = 0;
  nle      = 0;
  Aeq      = [];
  beq      = [];
  xlow     = -inf*ones(n,1);
  xupp     =  inf*ones(n,1);
  nonlconS = '';

elseif nargin == 6,
  Aeq      = varargin{1};
  beq      = varargin{2};
  nonlconS = '';

  [me,n0]  = size(Aeq);
  nli      = 0;
  nle      = 0;
  xlow     = -inf*ones(n,1);
  xupp     =  inf*ones(n,1);

elseif nargin == 8,
  Aeq      = varargin{1};
  beq      = varargin{2};
  xlow     = varargin{3};
  xupp     = varargin{4};
  nonlconS = '';

  [me,n0 ] = size(Aeq);
  nli      = 0;
  nle      = 0;

elseif nargin == 10,
  Aeq      = varargin{1};
  beq      = varargin{2};
  xlow     = varargin{3};
  xupp     = varargin{4};
  nonlcon  = varargin{5};

  nonlconS = func2str(nonlcon);

  [me,n0]  = size(Aeq);
  [c,ceq]  = feval(nonlcon,x0);
  nli      = size(c,1);
  nle      = size(ceq,1);

  if isempty(xlow),
    xlow   = -inf*ones(n,1);
    xupp   =  inf*ones(n,1);
  end

else
  error('Wrong number of input arguments')
end

nCon = 1 + nli + nle + mi + me;   % number of constraints (size of F(x))


% snoptA problem format:
%    minimize    F_obj (x)
%   such that  l_f <= F(x) <= u_f
%              l   <=   x  <= u
%
%           [ F_obj   ]            [ F_0'     ]   [  0    ]
%           [ c(x)    ]            [ c'(x)    ]   [  0    ]
%           [ c_eq(x) ]            [ c_eq'(x) ]   [  0    ]
%   F(x) =  [ Ax      ]    F'(x) = [ 0        ] + [  A    ]
%           [ A_eq x  ]            [ 0        ]   [  A_eq ]
%                                    "G(x)"         "A"

[iAfun,jAvar,Aij] = find( [ zeros(1+nli+nle,n); A; Aeq ]);
[iGfun,jGvar,Gij] = find( [ ones(1+nli+nle,n); zeros(mi+me,n) ]);

x       =  x0;
xmul    =  zeros(n,1);
xstate  =  zeros(n,1);
Fmul    =  zeros(nCon,1);
Fstate  =  zeros(nCon,1);
ObjAdd  =  0;
ObjRow  =  1;
Flow    = [ -inf; -inf*ones(nli,1); zeros(nle,1); -inf*ones(mi,1); zeros(me,1) ];
Fupp    = [  inf; zeros(nli,1); zeros(nle,1); b; beq ];

myobjS  = func2str(myobj); % SNOPT mex wants the string.
AA      = [ A ; Aeq ];


% Solve the problem!
solveopt = 1;
[x,F,xmul,Fmul,exitflag] = snoptcmex( solveopt,x,xlow,xupp,xmul,xstate, ...
				      Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow, ...
				      Aij,iAfun,jAvar,iGfun,jGvar, ...
				      '',myobjS,nonlconS,AA);

fval              = feval(myobj,x);
lambda.lower      = xmul;
lambda.upper      = xmul;
lambda.ineqnonlin = Fmul(2:nli+1);
lambda.eqnonlin   = Fmul(nli+2:nli+nle+1);
lambda.ineqlin    = Fmul(nli+nle+2:nli+nle+mi+1);
lambda.eqlin      = Fmul(nli+nle+mi+2:nCon);

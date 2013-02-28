function  [A,iAfun,jAvar,iGfun,jGvar] = snJac(usrfun,x0,xlow,xupp,nF)
%function [A,iAfun,jAvar,iGfun,jGvar] = snJac(usrfun,x0,xlow,xupp,nF)
%         Finds the coordinate structure for the Jacobian.

findJacOption = 17;

[A,iAfun,jAvar,iGfun,jGvar] = snoptcmex(findJacOption,usrfun,x0,xlow,xupp,nF);

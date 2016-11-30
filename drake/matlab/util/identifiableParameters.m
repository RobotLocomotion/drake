function [identifiable_params, coeff_mat, offsets,lin_params,beta] = identifiableParameters(expr,params)
% Given an expression (expr) and a set of parameters (params) computes
% a set of lumped parameters (identifiable_params), coefficients (coeff_mat), and
% offsets such that
%
%   expr = coeff_mat*identifiable_params + offsets
%
% where all columns of coeff_mat are linearly independent.

[lin_params,coeff_mat1,offsets] = linearParameters(expr,params);
[beta,independent_idx] = baseParameters(coeff_mat1);
identifiable_params = beta*lin_params;
coeff_mat = coeff_mat1(:,independent_idx);
end
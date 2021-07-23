
% Remove linearly dependent columns
[Csub, idx] = licols(C);

% Other matrices
dsub = d;
% Asub = A(:, idx);
Aeqsub = Aeq(:, idx);
lbsub = lb(idx);
% ubsub = ub(idx);

% Numbering of columns of C
nidx = zeros(1, size(C, 2));
nidx(idx) = 1;
nidx = find(nidx == 0);

% Remove rows that correspond to removed inequality constraint columns
% from c
%
%   nidx(nidx > Ncf + m) <- Indices corresponding to removed inequality
%   constraint columns
%   nidx(nidx > Ncf + m) - (Ncf + m) + n <- Rows corresponding to removed
%   inequality columns
rm_rows = ones(size(C, 1), 1, 'logical');
rm_rows(nidx(nidx > Ncf + m) - (Ncf + m) + n) = 0;

Csub = Csub(rm_rows, :);
dsub = d(rm_rows);

% Get optimal coefficients
[vars_ioc,rn_ioc,res_ioc,ef_ioc,out_ioc,~] = lsqlin(Csub,dsub,A,b,Aeqsub,beq,lbsub,ub);
% Extract difference quantities
alpha_ioc_sub = vars_ioc(1:Ncf);
alpha_ioc_sub'
[IneqNames, EqNames] = nl_constraint_names(itpParam,modelParam,LiftParam);
IneqNames(nidx-Ncf-m)
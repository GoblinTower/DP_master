% Simple test for verifying the grouping function works appropriately
clear, clc, close all;

addpath("..\");

%%%%%%%%%%%%%%%%%%
%%% First test %%%
%%%%%%%%%%%%%%%%%%
groups = [2,4,4];

% Input
u = [1,3,   4,5,6,1,    8,3,10,1;
     2,3,   7,3,8,1,    4,6,11,11];

% Expected results
exp_ceq = [-2;-1;   -1;4;-1;-5;5;7;     5;-2;-7;-5;9;0];

[c, ceq] = grouping(u, groups);
assert(isequal(exp_ceq, ceq), 'Expected outcome not obtained');

%%%%%%%%%%%%%%%%%%%
%%% Second test %%%
%%%%%%%%%%%%%%%%%%%'
groups = [1,4,1,2];

% Input
u = [1,   4,5,6,1,    8,    3,10;
     2,   7,3,8,1,    4,    6,11];

% Expected results
exp_ceq = [0;0;   -1;4;-1;-5;5;7;   0;0;    -7;-5];

[c, ceq] = grouping(u, groups);

assert(isequal(exp_ceq, ceq), 'Expected outcome not obtained');

disp('All tests successful');
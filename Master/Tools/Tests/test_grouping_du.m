% Simple test for verifying the grouping function works appropriately
clear, clc, close all;

addpath("..\");

%%%%%%%%%%%%%%%%%%
%%% First test %%%
%%%%%%%%%%%%%%%%%%
groups = [2,4,4];

um1 = [1;1];

% Input
u = [1,3,   4,5,6,1,    8,3,10,1;
     2,3,   7,3,8,1,    4,6,11,11];

% Expected results
exp_ceq = [2;0;   0;-8;0;9;-6;-12;     -12;-1;12;3;-16;-5];

[c, ceq] = grouping_du(u, groups, um1);
assert(isequal(exp_ceq, ceq), 'Expected outcome not obtained');

disp('All tests successful');
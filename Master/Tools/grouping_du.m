function [c,ceq] = grouping_du(u, groups, um1)
    
r = size(u,1);          % Number of input variables
h = size(u,2);          % Horizon
ng = length(groups);    % Number of groups

% Verify that group size and control input is equal
if (h ~= sum(groups))
    error('The group sizes does not match the input length')
end

% First convert u to delta u:
delta_u = [u(:,1) - um1, u(:,2:end) - u(:,1:end-1)];

% Inequality constraints
c = 0;

% Equality constraints
ceq = zeros(r*(h-ng),1);

% Grouping - Equality constraints
current_index = 1;
ceq_index = 1;
for i = 1:length(groups)
    g = groups(i);
    if (g == 1)
        ceq(ceq_index:ceq_index + r - 1) = zeros(r,1);
        current_index = current_index + 1;
        ceq_index = ceq_index + r;
    elseif (g > 1)
        diff = delta_u(:,current_index + 1:current_index + g - 1) - delta_u(:,current_index:current_index + g - 2);
        ceq(ceq_index:ceq_index + (g-1)*r - 1) = diff(:);
        current_index = current_index + g;
        ceq_index = ceq_index + (g-1)*r;
    else
        error('Input variables must be a set of positive integers');
    end
end

end
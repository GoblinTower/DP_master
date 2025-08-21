function J = smooth_cost_function(r, r1, w1, w2)
    if r < r1
        J = w1*r;
    else
        J = w1*r + w2*(r - r1)^2;
    end
end
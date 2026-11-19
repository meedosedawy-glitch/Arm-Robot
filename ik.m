function [v1, v2] = ik(px, py, l1, l2)
    D = (px^2 + py^2 - l1^2 - l2^2) / (2*l1*l2);
    S = -(1-D^2)^0.5;
    k1 = l1 + l2*D;
    k2 = l2*S;
    M = (k1*px + k2*py) / (k1^2 + k2^2);
    N = (k1*py - k2*px) / (k1^2 + k2^2);
    v1 = atan2(N, M);
    v2 = atan2(S, D);
end
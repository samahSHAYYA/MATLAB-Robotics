function test_ik_3link
% TEST_IK_3LINK Verify analytic IK produces correct joint angles
%   1. Pick a foot position
%   2. Compute IK -> joint angles
%   3. Compute FK from joint angles -> foot position
%   4. Compare: error should be near zero

    L1 = 0.2;  % upper leg length (m)
    L2 = 0.25; % lower leg length (m)

    % Test points (foot positions in leg base frame)
    testPoints = [
        0.15,  0.05, -0.30;  % reach forward-down
        0.00,  0.10, -0.35;  % straight down
       -0.10,  0.05, -0.25;  % reach back-down
        0.10,  0.00, -0.28;  % centered
    ];

    passes = 0;
    for i = 1:size(testPoints, 1)
        x = testPoints(i, 1);
        y = testPoints(i, 2);
        z = testPoints(i, 3);

        [theta1, theta2, theta3] = legIK(x, y, z, L1, L2);
        [xf, yf, zf] = legFK(theta1, theta2, theta3, L1, L2);

        error = norm([xf, yf, zf] - [x, y, z]);

        fprintf('Point %d: target=(%.3f,%.3f,%.3f)  FK=(%.3f,%.3f,%.3f)  error=%.2e\n', ...
                i, x, y, z, xf, yf, zf, error);

        if error < 1e-10
            passes = passes + 1;
            fprintf('  [PASS]\n');
        else
            fprintf('  [FAIL]\n');
        end
    end

    fprintf('\n%d / %d passed.\n', passes, size(testPoints, 1));
end

function [theta1, theta2, theta3] = legIK(x, y, z, L1, L2)
    r = sqrt(x^2 + y^2 + z^2);
    theta1 = atan2(y, z);

    cos_theta3 = (L1^2 + L2^2 - r^2) / (2 * L1 * L2);
    cos_theta3 = max(-1, min(1, cos_theta3));
    theta3 = acos(cos_theta3);

    theta2 = atan2(x, sqrt(y^2 + z^2)) - atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
end

function [x, y, z] = legFK(theta1, theta2, theta3, L1, L2)
    % Forward kinematics for 3-link leg
    % Returns foot position in leg base frame
    T1 = makehgtform('zrotate', theta1);
    T2 = makehgtform('yrotate', -theta2);
    T3 = makehgtform('yrotate', -theta3);

    % Foot in final knee frame
    foot = [L2; 0; 0; 1];

    % Transform to leg base
    p1 = T1 \ (T2 \ (T3 \ foot));  % This is wrong — compose forward
    p2 = T3 * [L2; 0; 0; 1];
    p3 = T2 * p2 + [0; 0; L1; 0];  % knee position in hip frame
    p4 = T1 * p3;  % foot in leg base frame

    x = p4(1); y = p4(2); z = p4(3);
end

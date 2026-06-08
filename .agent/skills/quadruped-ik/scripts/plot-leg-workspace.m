function plot_leg_workspace(L1, L2)
% PLOT_LEG_WORKSPACE Visualize reachable workspace of a 3-link leg
%   plot_leg_workspace(0.2, 0.25)  % default lengths
%
% Samples the joint space and plots all reachable foot positions.

    if nargin < 1, L1 = 0.2; end
    if nargin < 2, L2 = 0.25; end

    nSamples = 30;
    theta1Range = linspace(-0.5, 0.5, nSamples);
    theta2Range = linspace(-1.2, 1.2, nSamples);
    theta3Range = linspace(0.3, 2.8, nSamples);

    points = zeros(nSamples^3, 3);
    idx = 1;
    for t1 = theta1Range
        for t2 = theta2Range
            for t3 = theta3Range
                [x, y, z] = legFK(t1, t2, t3, L1, L2);
                points(idx, :) = [x, y, z];
                idx = idx + 1;
            end
        end
    end

    figure('Name', 'Leg Workspace', 'NumberTitle', 'off');
    plot3(points(:,1), points(:,2), points(:,3), '.', 'MarkerSize', 3);
    hold on;
    grid on;
    axis equal;
    xlabel('x (forward)'); ylabel('y (outward)'); zlabel('z (down)');
    title(sprintf('Leg workspace: L1=%.2f, L2=%.2f', L1, L2));

    % Draw leg in neutral position
    [x0, y0, z0] = legFK(0, 0.5, 1.0, L1, L2);
    plot3(0, 0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    plot3(x0, y0, z0, 'go', 'MarkerSize', 8, 'LineWidth', 2);

    legend({'Workspace', 'Hip', 'Foot (neutral)'}, 'Location', 'best');
    fprintf('Plotted %d reachable foot positions.\n', size(points, 1));
end

function [x, y, z] = legFK(theta1, theta2, theta3, L1, L2)
    % Simplified FK using rotation + translation
    % Hip position: [0; 0; 0] in leg frame
    % Knee position after hip abduction + hip flexion
    knee = [0; L1 * cos(theta2); L1 * sin(theta2)];
    knee = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1] * knee;

    % Foot relative to knee (in knee frame, rotated by knee angle)
    footLocal = [L2 * cos(theta3); 0; -L2 * sin(theta3)];
    % Rotate foot by hip flexion + hip abduction to world
    footWorld = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1] * ...
                [cos(theta2) 0 sin(theta2); 0 1 0; -sin(theta2) 0 cos(theta2)] * footLocal;
    footPos = knee + footWorld;

    x = footPos(1); y = footPos(2); z = footPos(3);
end

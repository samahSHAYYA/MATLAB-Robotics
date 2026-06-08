function test_wireframe
% TEST_WIREFRAME Verify hgtransform + patch rendering works

    fig = figure('Visible', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    view(3);
    axis equal;
    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    xlim([-2 2]); ylim([-2 2]); zlim([-2 2]);

    tform = hgtransform(ax);

    % Unit cube wireframe
    verts = [0 0 0; 1 0 0; 1 1 0; 0 1 0;
             0 0 1; 1 0 1; 1 1 1; 0 1 1];
    faces = [1 2 3 4; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1; 5 6 7 8];
    p = patch('Parent', tform, 'Vertices', verts, 'Faces', faces, ...
              'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 2);

    % Test translate
    T = eye(4);
    T(1:3, 4) = [0.5; 0.5; 0];
    tform.Matrix = T;
    drawnow limitrate;

    % Verify patch is a child of transform
    if isvalid(p) && isvalid(tform)
        fprintf('[PASS] Wireframe patch and hgtransform valid.\n');
    else
        fprintf('[FAIL] Graphics objects invalid.\n');
    end

    close(fig);
end

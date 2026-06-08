function test_drawnow
% TEST_DRAWNOW Validate that drawnow limitrate is used and performs well

    fig = figure('Visible', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    view(3);
    axis equal;

    tform = hgtransform(ax);
    line('Parent', tform, 'XData', [0 1], 'YData', [0 0], 'ZData', [0 0], ...
         'Color', 'r', 'LineWidth', 2);

    numFrames = 100;
    tic;
    for i = 1:numFrames
        T = eye(4);
        T(1:3, 4) = [i*0.01; 0; 0];
        tform.Matrix = T;
        drawnow limitrate;
    end
    elapsed = toc;
    fps = numFrames / elapsed;

    fprintf('Animation test: %d frames in %.3f s = %.1f FPS\n', ...
            numFrames, elapsed, fps);

    if fps >= 30
        fprintf('[PASS] Frame rate above 30 FPS.\n');
    elseif fps >= 15
        fprintf('[WARN] Frame rate below 30 FPS but functional.\n');
    else
        fprintf('[FAIL] Frame rate unusably low.\n');
    end

    close(fig);
end

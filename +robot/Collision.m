classdef Collision
    %COLLISION  Static collision detection via Separating Axis Theorem.
    %   Supports OBB (oriented bounding box) vs OBB tests.

    methods (Static)
        function hit = checkOBB(centerA, quatA, halfA, centerB, quatB, halfB)
            %CHECKOBB  OBB-vs-OBB overlap test using SAT.
            %   hit = robot.Collision.checkOBB(cA, qA, hA, cB, qB, hB)
            %   Inputs: centerA - 3x1 world position of box A
            %           quatA   - 4x1 quaternion [w;x;y;z]' of box A
            %           halfA   - 3x1 half-extents of box A in body frame
            %           centerB, quatB, halfB - same for box B
            %   Outputs: hit - logical, true if boxes overlap
            %
            %   Tests all 15 potential separating axes:
            %     3 face normals from A, 3 from B, 9 edge cross-products.

            RA = robot.Utils.quatToRotmx(quatA);
            RB = robot.Utils.quatToRotmx(quatB);

            d = centerB(:) - centerA(:);

            % Build list of test axes (unique, normalized)
            axes = zeros(3, 15);
            k = 0;
            for i = 1:3
                k = k + 1; axes(:, k) = RA(:, i);
                k = k + 1; axes(:, k) = RB(:, i);
            end
            for i = 1:3
                for j = 1:3
                    ax = cross(RA(:,i), RB(:,j));
                    if norm(ax) > 1e-10
                        k = k + 1; axes(:, k) = ax / norm(ax);
                    end
                end
            end
            axes = axes(:, 1:k);

            for k = 1:size(axes, 2)
                n = axes(:,k);

                rA = halfA(1) * abs(dot(RA(:,1), n)) + ...
                     halfA(2) * abs(dot(RA(:,2), n)) + ...
                     halfA(3) * abs(dot(RA(:,3), n));
                rB = halfB(1) * abs(dot(RB(:,1), n)) + ...
                     halfB(2) * abs(dot(RB(:,2), n)) + ...
                     halfB(3) * abs(dot(RB(:,3), n));

                if abs(dot(d, n)) > rA + rB + 1e-12
                    hit = false;
                    return;
                end
            end

            hit = true;
        end

        function [center, half] = robotOBB(rbt)
            %ROBOTTOBB  Return the OBB centre and half-extents for a robot.
            %   [center, half] = robot.Collision.robotOBB(rbt)
            %   Uses robot State(1:3) for position, State(4:7) for
            %   orientation, and geometry-dependent half-extents.
            %
            %   Supported robot types:
            %     Quadcopter        — bodySize/2
            %     Quadruped         — [bodyLength, bodyWidth, bodyHeight]/2
            %     DifferentialDrive — [0.2, 0.15, 0.05]
            center = rbt.State(1:3);

            if isa(rbt, 'robot.Quadcopter')
                half = rbt.bodySize(:) / 2;
            elseif isa(rbt, 'robot.Quadruped')
                half = [rbt.bodyLength; rbt.bodyWidth; rbt.bodyHeight] / 2;
            elseif isa(rbt, 'robot.DifferentialDrive')
                half = [0.2; 0.15; 0.05];
            else
                half = [0.1; 0.1; 0.1];
            end
        end
        function pairs = checkAll(robots, useParallel)
            %CHECKALL  Check all robot pairs for collisions.
            %   pairs = robot.Collision.checkAll(robots) checks N robots
            %   for OBB collisions across all N*(N-1)/2 unique pairs.
            %
            %   Inputs:  robots - 1xN cell array of robot handles
            %            useParallel - (optional) true => parfor
            %   Outputs: pairs - NxN logical matrix, pairs(i,j)=true if
            %            robot i and robot j collide (upper-triangle).
            %
            %   Each pair is checked independently.  When the Parallel
            %   Computing Toolbox is available and useParallel is true,
            %   pairs are dispatched across workers for speed.

            if nargin < 2
                useParallel = false;
            end
            N = numel(robots);
            pairs = false(N, N);

            % Pre-extract OBB data as numeric arrays for parfor slicing
            cenM = zeros(3, N); quaM = zeros(4, N); halM = zeros(3, N);
            for i = 1:N
                [c, h] = robot.Collision.robotOBB(robots{i});
                cenM(:, i) = c;
                halM(:, i) = h;
                quaM(:, i) = robots{i}.State(4:7);
            end

            % Build pair list
            nPairs = N*(N-1)/2;
            iList = zeros(nPairs,1);
            jList = zeros(nPairs,1);
            idx = 0;
            for i = 1:N-1
                for j = i+1:N
                    idx = idx + 1;
                    iList(idx) = i;
                    jList(idx) = j;
                end
            end

            results = false(nPairs,1);

            % Build pair-indexed arrays so parfor can slice them
            cA = zeros(3, nPairs); qA = zeros(4, nPairs); hA = zeros(3, nPairs);
            cB = zeros(3, nPairs); qB = zeros(4, nPairs); hB = zeros(3, nPairs);
            for k = 1:nPairs
                ii = iList(k); jj = jList(k);
                cA(:,k) = cenM(:, ii); qA(:,k) = quaM(:, ii); hA(:,k) = halM(:, ii);
                cB(:,k) = cenM(:, jj); qB(:,k) = quaM(:, jj); hB(:,k) = halM(:, jj);
            end

            if useParallel
                try
                    parfor k = 1:nPairs
                        results(k) = robot.Collision.checkOBB(...
                            cA(:,k), qA(:,k), hA(:,k), cB(:,k), qB(:,k), hB(:,k));
                    end
                catch ME
                    warning('Collision:ParforFailed', ...
                        'parfor failed (%s), falling back to serial.', ME.message);
                    for k = 1:nPairs
                        results(k) = robot.Collision.checkOBB(...
                            cA(:,k), qA(:,k), hA(:,k), cB(:,k), qB(:,k), hB(:,k));
                    end
                end
            else
                for k = 1:nPairs
                    results(k) = robot.Collision.checkOBB(...
                        cA(:,k), qA(:,k), hA(:,k), cB(:,k), qB(:,k), hB(:,k));
                end
            end

            for k = 1:nPairs
                pairs(iList(k), jList(k)) = results(k);
            end
        end
    end
end

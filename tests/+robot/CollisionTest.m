classdef CollisionTest < matlab.unittest.TestCase

    properties
        halfSize = [0.5; 0.3; 0.2]
        identityQuat = [1; 0; 0; 0]
    end

    methods (Test)

        function separatedAlongX(testCase)
            hit = robot.Collision.checkOBB([0;0;0], testCase.identityQuat, testCase.halfSize, ...
                                           [2;0;0],  testCase.identityQuat, testCase.halfSize);
            testCase.verifyFalse(hit);
        end

        function overlapping(testCase)
            hit = robot.Collision.checkOBB([0;0;0], testCase.identityQuat, testCase.halfSize, ...
                                           [0.3;0;0], testCase.identityQuat, testCase.halfSize);
            testCase.verifyTrue(hit);
        end

        function touchingIsContact(testCase)
            hit = robot.Collision.checkOBB([0;0;0], testCase.identityQuat, testCase.halfSize, ...
                                           [1.0;0;0], testCase.identityQuat, testCase.halfSize);
            testCase.verifyTrue(hit);
        end

        function barelySeparated(testCase)
            hit = robot.Collision.checkOBB([0;0;0], testCase.identityQuat, testCase.halfSize, ...
                                           [1.001;0;0], testCase.identityQuat, testCase.halfSize);
            testCase.verifyFalse(hit);
        end

        function barelyOverlapping(testCase)
            hit = robot.Collision.checkOBB([0;0;0], testCase.identityQuat, testCase.halfSize, ...
                                           [0.99;0;0], testCase.identityQuat, testCase.halfSize);
            testCase.verifyTrue(hit);
        end

        function sameBox(testCase)
            hit = robot.Collision.checkOBB([0;0;0], testCase.identityQuat, testCase.halfSize, ...
                                           [0;0;0], testCase.identityQuat, testCase.halfSize);
            testCase.verifyTrue(hit);
        end

        function rotatedSeparated(testCase)
            q45 = [cos(pi/8); 0; 0; sin(pi/8)];
            hit = robot.Collision.checkOBB([0;0;0], q45, testCase.halfSize, ...
                                           [0;1.5;0], testCase.identityQuat, testCase.halfSize);
            testCase.verifyFalse(hit);
        end

        function rotatedOverlapping(testCase)
            q45 = [cos(pi/8); 0; 0; sin(pi/8)];
            hit = robot.Collision.checkOBB([0;0;0], q45, testCase.halfSize, ...
                                           [0.3;0.3;0], testCase.identityQuat, testCase.halfSize);
            testCase.verifyTrue(hit);
        end

        function edgeEdgeOverlap(testCase)
            qB = robot.CollisionTest.quatFromAxisAngle(pi/4, [0;1;0]);
            hit = robot.Collision.checkOBB([-0.1;0;0.1], testCase.identityQuat, testCase.halfSize, ...
                                           [0.1;0;-0.1], qB, testCase.halfSize);
            testCase.verifyTrue(hit);
        end

        function robotOBBQuadcopter(testCase)
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            r = robot.Quadcopter(p);
            [c, h] = robot.Collision.robotOBB(r);
            L = r.armLength;
            eh = [r.bodySize(1)/2 + L; r.bodySize(2)/2 + L; r.bodySize(3)/2];
            testCase.verifyEqual(c, r.State(1:3));
            testCase.verifyEqual(h, eh);
        end

        function robotOBBQuadruped(testCase)
            p = struct();
            p.geometric.bodyLength = 0.4;
            p.geometric.bodyWidth = 0.2;
            p.geometric.bodyHeight = 0.1;
            p.kinematic.legLength1 = 0.15;
            p.kinematic.legLength2 = 0.15;
            p.geometric.shoulderWidth = 0.12;
            p.dynamic.mass = 3;
            p.dynamic.inertia = diag([0.015, 0.04, 0.05]);
            p.elastic.k_contact = 5000;
            p.elastic.b_contact = 50;
            r = robot.Quadruped(p);
            [c, h] = robot.Collision.robotOBB(r);
            legExt = r.legLength1 + r.legLength2;
            ec = r.State(1:3); ec(3) = ec(3) - legExt/2;
            eh = [r.bodyLength/2; max(r.bodyWidth, r.shoulderWidth)/2; ...
                  (r.bodyHeight + legExt)/2];
            testCase.verifyEqual(c, ec);
            testCase.verifyEqual(h, eh);
        end

        function checkAllOverlapDetection(testCase)
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            rA = robot.Quadcopter(p); rA.State(1:3) = [0;0;0];
            rB = robot.Quadcopter(p); rB.State(1:3) = [0.01;0;0];
            rC = robot.Quadcopter(p); rC.State(1:3) = [10;0;0];
            pAll = robot.Collision.checkAll({rA, rB, rC});
            testCase.verifyTrue(pAll(1,2));
            testCase.verifyFalse(pAll(1,3));
            testCase.verifyFalse(pAll(2,3));
        end

        function checkAllParallel(testCase)
            try
                pool = gcp('nocreate');
            catch
                pool = [];
            end
            testCase.assumeNotEmpty(pool, ...
                'Parallel pool required — skipping');
            p = struct();
            p.geometric.armLength = 0.2;
            p.geometric.bodySize = [0.1, 0.1, 0.05];
            p.dynamic.mass = 0.5;
            p.dynamic.inertia = diag([0.002, 0.002, 0.004]);
            p.dynamic.maxThrust = 2.0;
            rA = robot.Quadcopter(p); rA.State(1:3) = [0;0;0];
            rB = robot.Quadcopter(p); rB.State(1:3) = [0.01;0;0];
            rC = robot.Quadcopter(p); rC.State(1:3) = [10;0;0];
            pSeq = robot.Collision.checkAll({rA, rB, rC}, false);
            pPar = robot.Collision.checkAll({rA, rB, rC}, true);
            testCase.verifyEqual(pPar, pSeq);
        end

    end

    methods (Static, Access = private)
        function q = quatFromAxisAngle(angle, axis)
            axis = axis(:) / norm(axis);
            q = [cos(angle/2); axis * sin(angle/2)];
        end
    end

end

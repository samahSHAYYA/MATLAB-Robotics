function results = runAll
    addpath('tests');
    diary('test_output.txt');
    suite = matlab.unittest.TestSuite.fromPackage('robot', 'IncludingSubpackages', true);
    fleetSuite = matlab.unittest.TestSuite.fromClass(?RobotFleetAppTest);
    suite = [suite; fleetSuite];
    results = run(suite);
    diary('off');
    total = numel(results);
    passed = nnz([results.Passed]);
    failed = nnz([results.Failed]);
    fprintf('\n=== %d/%d passed, %d failed ===\n', passed, total, failed);
    if failed > 0
        fprintf('\nFAILED:\n');
        for i = find([results.Failed])
            fprintf('  %s\n', results(i).Name);
        end
    else
        delete('test_output.txt');
    end
end

function results = runAll
    addpath('tests');
    suite = matlab.unittest.TestSuite.fromPackage('robot', 'IncludingSubpackages', true);
    results = run(suite);
    total = numel(results);
    passed = nnz([results.Passed]);
    failed = nnz([results.Failed]);
    fprintf('\n=== %d/%d passed, %d failed ===\n', passed, total, failed);
    if failed > 0
        fprintf('\nFAILED:\n');
        for i = find([results.Failed])
            fprintf('  %s\n', results(i).Name);
        end
    end
end

function results = runAll
    addpath('tests');
    suite = testsuite('tests');
    results = run(suite);
    disp(table(results));
    total = sum([results.Details.Duration]);
    fprintf('\nTotal: %d passed, %d failed, %d incomplete (%.2f s)\n', ...
        nnz([results.Passed]), nnz([results.Failed]), nnz([results.Incomplete]), total);
    if any([results.Failed])
        fprintf('\nFAILED TESTS:\n');
        for i = find([results.Failed])
            fprintf('  %s: %s\n', results(i).Name, results(i).Details.DiagnosticRecord);
        end
    end
end

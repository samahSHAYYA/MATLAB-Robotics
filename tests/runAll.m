function results = runAll
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'tests'));
    diaryFile = fullfile(root, 'test_output.txt');
    diary(diaryFile);
    testFolder = fullfile(root, 'tests', '+robot');
    suite = matlab.unittest.TestSuite.fromFolder(testFolder, 'IncludingSubfolders', true);
    fleetFile = fullfile(root, 'tests', 'RobotFleetAppTest.m');
    if isfile(fleetFile)
        fleetSuite = matlab.unittest.TestSuite.fromFile(fleetFile);
        suite = [suite(:); fleetSuite(:)];
    end
    if isempty(suite)
        fprintf('No tests found.\n');
        results = [];
        diary('off');
        return;
    end
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
        if isfile(diaryFile); delete(diaryFile); end
    end
end

function results = lintCode
    src = dir('+robot/*.m');
    allOk = true;
    results = {};
    for i = 1:numel(src)
        f = fullfile(src(i).folder, src(i).name);
        msgs = checkcode(f, '-struct');
        if ~isempty(msgs)
            allOk = false;
            results{end+1} = struct('file', f, 'messages', msgs);
            fprintf('%s:\n', src(i).name);
            for j = 1:numel(msgs)
                fprintf('  Line %d, Col %d [%s]: %s\n', ...
                    msgs(j).line, msgs(j).column, msgs(j).message);
            end
        end
    end
    if allOk
        fprintf('All %d files lint-free.\n', numel(src));
    else
        error('Lint failed on %d file(s).', numel(results));
    end
end

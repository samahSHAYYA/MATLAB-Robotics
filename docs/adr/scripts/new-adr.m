function new_adr(title)
% NEW_ADR Scaffold a new Architecture Decision Record
%   new_adr('title-in-kebab-case')
%
% Creates docs/adr/NNNN-title-in-kebab-case.md from the template file.
% Auto-increments the ADR number based on existing files.

    if nargin < 1
        error('Usage: new_adr(''quaternion-state-model'')');
    end

    adrDir = fullfile(fileparts(fileparts(mfilename('fullpath'))));
    templateFile = fullfile(adrDir, 'templates', 'adr-template.md');

    if ~exist(templateFile, 'file')
        error('ADR template not found at:\n  %s', templateFile);
    end

    % Find the highest existing ADR number
    pattern = fullfile(adrDir, '*.md');
    files = dir(pattern);
    maxNum = 0;
    for i = 1:length(files)
        name = files(i).name;
        tokens = regexp(name, '^(\d{4})-', 'tokens');
        if ~isempty(tokens)
            num = str2double(tokens{1}{1});
            if num > maxNum, maxNum = num; end
        end
    end
    nextNum = maxNum + 1;

    if nextNum > 9999
        error('ADR number overflow (max 9999)');
    end

    newName = sprintf('%04d-%s.md', nextNum, title);
    newPath = fullfile(adrDir, newName);

    % Read template and write
    template = fileread(templateFile);
    template = strrep(template, '{NNNN}', sprintf('%04d', nextNum));
    template = strrep(template, '{Title}', strrep(title, '-', ' '));
    template = strrep(template, '{YYYY-MM-DD}', datestr(now, 'yyyy-mm-dd'));

    fid = fopen(newPath, 'w');
    fprintf(fid, '%s', template);
    fclose(fid);

    fprintf('Created ADR: %s\n', newPath);
end

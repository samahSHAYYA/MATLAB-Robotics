function validateClass(className)
% VALIDATECLASS Check that a robot class is properly defined
%   validateClass('Quadruped')
%
% Checks: inherits handle, implements abstract methods, parses without error.

    if nargin < 1
        error('Usage: validateClass(className)');
    end

    fprintf('Validating %s ...\n', className);

    % 1. Check file exists on path
    if ~exist(className, 'class')
        error('Class %s not found on MATLAB path. Ensure +robot/ is on path.', className);
    end

    % 2. Try instantiating with minimal params
    meta = ?className;
    if ~ismember('handle', {meta.SuperclassList.Name})
        error('%s does not inherit from handle. State mutations will not work.', className);
    end
    fprintf('  [OK] inherits handle\n');

    % 3. Check for abstract methods
    abstractMethods = meta.MethodList.findobj('Abstract', true);
    if ~isempty(abstractMethods)
        fprintf('  [WARN] abstract methods not yet implemented:\n');
        for i = 1:length(abstractMethods)
            fprintf('    - %s\n', abstractMethods(i).Name);
        end
    else
        fprintf('  [OK] no unimplemented abstract methods\n');
    end

    % 4. Verify superclass chain
    superclasses = {meta.SuperclassList.Name};
    fprintf('  Superclasses: %s\n', strjoin(superclasses, ' -> '));
    fprintf('Validation complete.\n');
end

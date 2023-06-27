%% Kinematics test
% Specify the number of tests
numTests = 10;

% Specify the tolerance for comparison
tolerance = 1e-6;

% serial link object
robot = actual;

%% Direct Kinematics tets
% Initialize a flag to track if all tests pass
allTestsPass = true;

% Loop over the number of tests
for i = 1:numTests
    % Generate random joint angles within the valid range, here we are
    % taking [-pi pi], full range.
    q = rand(1, size(robot.links, 2)) .* (robot.qlim(:,2) - robot.qlim(:,1))' + robot.qlim(:,1)';

    % Compute the forward kinematics using our implementation
    Tee_computed = direct_kine(dh, q);

    % Compute the forward kinematics using the toolbox
    Tee_toolbox = double(robot.fkine(q));

    % Check if the computed forward kinematics results are almost equal
    if ~isequal(abs(Tee_computed - Tee_toolbox) < tolerance, ones(size(Tee_computed)))
        fprintf("Direct Kinematics Test %d failed.\n", i);
        allTestsPass = false;
    else
        fprintf("Direct Kinematics %d passed.\n", i);
    end
end

% Check if all tests pass
if allTestsPass
    fprintf("All direct kinematics tests passed.\n");
else
    fprintf("Some/All direct kinematics tests failed.\n");
end

%% Inverse kinematics test
% Initialize a flag to track if all tests pass
allTestsPass = true;

% Loop over the number of tests
for i = 1:numTests
    % Generate random joint angles within the valid range
    q = rand(1, size(robot.links, 2)) .* (robot.qlim(:,2) - robot.qlim(:,1))' + robot.qlim(:,1)';

    % Compute the forward kinematics using our implementation
    Tee_computed = direct_kine(dh, q);

    % Compute the inverse kinematics using our implementation
    qc = inv_kine(Tee_computed, dh, q);

    % Compute the forward kinematics using the computed inverse kinematics
    Tee_ikine_computed = direct_kine(dh, qc);

    % Check if the computed inverse kinematics results are within tolerance
    if ~isequal(abs(Tee_computed - Tee_ikine_computed) < tolerance, ones(size(Tee_computed)))
        disp(['Inverse Kinematics Test ', num2str(i), ' failed.']);
        allTestsPass = false;
    else
        disp(['Inverse Kinematics Test ', num2str(i), ' passed.']);
    end
end

% Check if all tests pass
if allTestsPass
    disp('All inverse kinematics tests passed.');
else
    disp('Some inverse kinematics tests failed.');
end


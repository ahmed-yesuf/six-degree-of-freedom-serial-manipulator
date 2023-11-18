function initialize
% Initialize rtb toolbox and add the files to MATLAB PATH
addpath(genpath('lib/'))
run('startup_rvc.m')

% test trajectory
test_trajectory('j', 1);
end

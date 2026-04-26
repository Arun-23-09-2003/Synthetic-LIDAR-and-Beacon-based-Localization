function project_root = setup_project_paths()
%SETUP_PROJECT_PATHS Add project folders to the MATLAB path.

project_root = fileparts(mfilename('fullpath'));
addpath(project_root);
addpath(genpath(fullfile(project_root, 'config')));
addpath(genpath(fullfile(project_root, 'src')));
end

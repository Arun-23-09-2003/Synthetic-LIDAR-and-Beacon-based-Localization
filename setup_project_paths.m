function project_root = setup_project_paths()
%SETUP_PROJECT_PATHS Add project folders to the MATLAB path.

project_root = fileparts(mfilename('fullpath'));
addpath(genpath(project_root));
end

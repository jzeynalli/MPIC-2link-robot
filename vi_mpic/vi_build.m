function vi_build(compile_toolbox)
%VI_BUILD Compile the VI-MPIC problem into vi_mpic/+CmexFiles.
%   vi_build(true) also rebuilds the GRAMPC toolbox in the submodule.
%   The package lives in this folder, so run the VI scripts from here and
%   keep the repository root (with its own +CmexFiles) off the path.

if nargin < 1
    compile_toolbox = false;
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));

grampc_root = '../external/grampc/';
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));
if compile_toolbox || ~exist([grampc_root 'matlab/bin'], 'dir')
    grampc_make_toolbox(grampc_root);
end
grampc_make_probfct(grampc_root, 'probfct_VI_MPIC.c');
end

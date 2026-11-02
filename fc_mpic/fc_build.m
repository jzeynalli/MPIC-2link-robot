function fc_build(compile_toolbox)
%FC_BUILD Compile the GRAMPC problems of the FC-MPIC study.
%   probfct_FC_MPIC.c (classical MPIC, FC-MPIC and its ablations) goes into
%   the package +CmexFC, ../vi_mpic/probfct_VI_MPIC.c (VI-MPIC, first
%   draft) into +CmexVI, both in this folder, so that one closed loop can
%   call either. fc_build(true) also rebuilds the GRAMPC toolbox.

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
clear mex %#ok<CLMEX> loaded MEX files are locked on Windows
make(grampc_root, 'probfct_FC_MPIC.c', '+CmexFC');
make(grampc_root, '../vi_mpic/probfct_VI_MPIC.c', '+CmexVI');
rehash path
end

function make(grampc_root, src, pkg)
% grampc_make_probfct always writes the package +CmexFiles
grampc_make_probfct(grampc_root, src);
if exist(pkg, 'dir')
    rmdir(pkg, 's');
end
movefile('+CmexFiles', pkg);
end

function T = run_vi_sweep(task, names, speeds, iters, offsets, file)
%RUN_VI_SWEEP Closed-loop runs for a list of variants; writes one CSV row per run.
%   T = run_vi_sweep(task, names, speeds, iters, offsets, file)
%   task     'impact' (hold, approach, press; the first-draft benchmark) or
%            'slide' (the same, then slide over a wavy wall, E3 of the notes)
%   names    cell array of variant names (vi_variants)
%   speeds   approach speeds [m/s], iters gradient iterations per MPC step,
%   offsets  true minus believed wall position [m]
%   file     CSV file to write (results/...)
%
%   Several sweeps can run in parallel MATLAB processes; vi_report_v2 merges
%   their files.

here = fileparts(mfilename('fullpath'));
old = cd(here);
cleanup = onCleanup(@() cd(old));
addpath(fullfile(here, '..', 'external', 'grampc', 'matlab', 'mfiles'));

P0 = vi_params();
if strcmp(task, 'slide')
    P0.task.slide.enable = true;
    P0.task.T_sim = 4.0;
end
V = vi_variants(P0, names);

rows = cell(0, 22);
for it = iters
    for v = speeds
        P = P0;
        P.task.v_app = v;
        P.mpc.MaxGradIter = it;
        for i = 1:numel(V)
            for j = 1:numel(offsets)
                R = vi_simulate(P, V(i), offsets(j));
                m = R.metrics;
                rows(end + 1, :) = {task, V(i).name, V(i).family, v, it, 1e3 * offsets(j), ...
                    1e3 * m.e_track_rms, 1e3 * m.e_track_max, m.contact, m.t_approach, m.F_peak, ...
                    m.impact_peak, m.t_settle, m.F_rmse_end, m.k_contact, m.cpu_med, m.cpu_max, ...
                    m.F_rmse_slide, m.F_dev_slide, 1e3 * m.y_rmse_slide, m.k_slide, ...
                    double(isfield(V(i).opt, 'reflex') && V(i).opt.reflex)}; %#ok<AGROW>
            end
            fprintf('%s  iter %d  v_app %.1f  %-22s done\n', task, it, v, V(i).name);
        end
    end
end
T = cell2table(rows, 'VariableNames', {'task', 'name', 'family', 'v_app', 'iterations', 'offset_mm', ...
    'e_track_rms_mm', 'e_track_max_mm', 'contact', 't_approach_s', 'F_peak_N', 'impact_peak_N', ...
    't_settle_s', 'F_rmse_end_N', 'k_contact', 'cpu_med_ms', 'cpu_max_ms', 'F_rmse_slide_N', ...
    'F_dev_slide_N', 'y_rmse_slide_mm', 'k_slide', 'reflex'});
writetable(T, file);
end

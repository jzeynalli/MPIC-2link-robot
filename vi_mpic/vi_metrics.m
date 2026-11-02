function M = vi_metrics(L, Fn, P, t_slide)
%VI_METRICS Performance measures of one closed-loop run.
%   e_track_rms/max  EE position error in the disturbance window [m]
%   t_approach       start of the approach to first contact [s]
%   F_peak           largest contact force [N]
%   overshoot        max(0, F_peak - F_des) [N]
%   impact_peak      largest force within 100 ms after first contact [N]
%   t_settle         first contact to |F - F_des| <= 1 N for good [s]
%   F_rmse_end       RMS force error over the last 0.5 s [N]
%   k_contact        normal stiffness at first contact [N/m]
%   cpu_med/cpu_max  GRAMPC time per step [ms]
%   With the sliding task (t_slide finite), over the sliding time except
%   its first 0.1 s:
%   F_rmse_slide     RMS force error [N]
%   F_dev_slide      largest |F - F_des| [N]
%   y_rmse_slide     RMS error of the tangential position [m]
%   k_slide          mean normal stiffness [N/m]

if nargin < 4
    t_slide = inf;
end
Fd = P.task.F_des;
w = L.t >= P.task.track_window(1) & L.t <= P.task.track_window(2);
e = sqrt(sum((L.p(w, :) - P.task.pA').^2, 2));
M.e_track_rms = sqrt(mean(e.^2));
M.e_track_max = max(e);

ic = find(Fn.F > P.F_contact & Fn.t >= P.task.t_app, 1);
M.contact = ~isempty(ic);
if M.contact
    tc = Fn.t(ic);
    M.t_approach = tc - P.task.t_app;
    M.impact_peak = max(Fn.F(Fn.t >= tc & Fn.t <= tc + 0.1));
    off = find(abs(Fn.F - Fd) > 1 & Fn.t >= tc, 1, 'last');
    if isempty(off)
        M.t_settle = 0;
    elseif off < numel(Fn.F)
        M.t_settle = Fn.t(off + 1) - tc;
    else
        M.t_settle = NaN;
    end
    [~, kc] = min(abs(L.t - tc));
    M.k_contact = L.k(kc, 1);
else
    M.t_approach = NaN;
    M.impact_peak = NaN;
    M.t_settle = NaN;
    M.k_contact = NaN;
end
M.F_peak = max(Fn.F);
M.overshoot = max(0, M.F_peak - Fd);
M.F_rmse_end = sqrt(mean((Fn.F(Fn.t >= P.task.T_sim - 0.5) - Fd).^2));
M.cpu_med = median(L.cpu);
M.cpu_max = max(L.cpu);

M.F_rmse_slide = NaN;
M.F_dev_slide = NaN;
M.y_rmse_slide = NaN;
M.k_slide = NaN;
if isfinite(t_slide)
    ws = Fn.t >= t_slide + 0.1 & Fn.t <= t_slide + P.task.slide.dur;
    wc = L.t >= t_slide + 0.1 & L.t <= t_slide + P.task.slide.dur;
    if any(ws)
        M.F_rmse_slide = sqrt(mean((Fn.F(ws) - Fd).^2));
        M.F_dev_slide = max(abs(Fn.F(ws) - Fd));
        M.y_rmse_slide = sqrt(mean((L.p(wc, 2) - L.pdes(wc, 2)).^2));
        M.k_slide = mean(L.k(wc, 1));
    end
end
end

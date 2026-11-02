function M = fc_metrics(L, Fn, P)
%FC_METRICS Performance measures of one sanding run, from P.task.t_metric on.
%   Computed from the 1 kHz plant log Fn (true positions and forces):
%   et_rms/max/rev/mid  tangential tracking error py - py* [mm]: RMS, largest,
%                       RMS in the reversal windows (|dpy*/dt| below
%                       rev_frac of its peak) and RMS in mid-stroke
%   efn_rms/max/rev/mid normal force error fn - F_des [N], the same windows
%   lost                share of the time without contact
%   fn_pos/fn_neg       mean normal force in mid-stroke, moving in +y / -y [N]
%   ft_pos/ft_neg       mean friction force, the same windows [N]
%   rn                  fn_pos / fn_neg, the normal-force ratio of Prop. 2.4
%   mu_end              friction estimate at the end; capped: steps with mu_k capped
%   cpu_med/cpu_max     GRAMPC time per step [ms]

Tk = P.task;
w = 2 * pi / Tk.T;
tau = Fn.t - Tk.t0;
ys = Tk.y0 + (tau > 0) .* Tk.A .* (1 - cos(w * tau));
vs = (tau > 0) .* Tk.A * w .* sin(w * tau);
win = Fn.t >= Tk.t_metric;
rev = win & abs(vs) < Tk.rev_frac * Tk.A * w;
mid = win & ~rev;
et = 1e3 * (Fn.p(:, 2) - ys);
ef = Fn.F(:, 1) - Tk.F_des;
rms = @(a, m) sqrt(mean(a(m).^2));

M.et_rms  = rms(et, win);
M.et_max  = max(abs(et(win)));
M.et_rev  = rms(et, rev);
M.et_mid  = rms(et, mid);
M.efn_rms = rms(ef, win);
M.efn_max = max(abs(ef(win)));
M.efn_rev = rms(ef, rev);
M.efn_mid = rms(ef, mid);
M.lost    = mean(Fn.F(win, 1) <= 0);
pos = mid & vs > 0;
neg = mid & vs < 0;
M.fn_pos  = mean(Fn.F(pos, 1));
M.fn_neg  = mean(Fn.F(neg, 1));
M.ft_pos  = mean(Fn.F(pos, 2));
M.ft_neg  = mean(Fn.F(neg, 2));
M.rn      = M.fn_pos / M.fn_neg;
M.mu_end  = L.muhat(end);
M.capped  = sum(L.capped);
M.cpu_med = median(L.cpu);
M.cpu_max = max(L.cpu);
end

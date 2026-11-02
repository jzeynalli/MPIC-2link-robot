function animateTwoLinkPlanar(t, X, p, anim)
if nargin < 4 || isempty(anim), anim = struct(); end
if ~isfield(anim,'fps'),          anim.fps          = 60;           end
if ~isfield(anim,'showTrace'),    anim.showTrace     = true;        end
if ~isfield(anim,'traceMode'),    anim.traceMode     = 'tail';      end
if ~isfield(anim,'traceLength'),  anim.traceLength   = 250;         end
if ~isfield(anim,'axisPadding'),  anim.axisPadding   = 0.1;         end
if ~isfield(anim,'title'),        anim.title         = '2-Link Planar Arm'; end
if ~isfield(anim,'record'),       anim.record        = false;       end
if ~isfield(anim,'outputFile'),   anim.outputFile    = 'animation'; end
if ~isfield(anim,'videoFormat'),  anim.videoFormat   = 'MPEG-4';   end
if ~isfield(anim,'videoQuality'), anim.videoQuality  = 85;          end

L1 = p.length(1);  L2 = p.length(2);
q1 = X(:,1);       q2 = X(:,2);
N  = numel(t);
P1 = zeros(N,2);  P2 = zeros(N,2);
for k = 1:N
    th1  = q1(k);
    th12 = q1(k) + q2(k);
    P1(k,:) = [L1*cos(th1),  L1*sin(th1)];
    P2(k,:) = P1(k,:) + [L2*cos(th12), L2*sin(th12)];
end

reach = L1 + L2;
pad   = anim.axisPadding;
axlim = [-reach-pad, reach+pad, -reach-pad, reach+pad];

tRender = (t(1) : 1/anim.fps : t(end))';
idx     = interp1(t, (1:N)', tRender, 'nearest', 'extrap');

fig = figure('Color','w');
ax  = axes(fig);
axis(ax,'equal');  axis(ax,axlim);  grid(ax,'on');
xlabel(ax,'x [m]');  ylabel(ax,'y [m]');  title(ax, anim.title);
ax.XTickMode = 'auto';  ax.XTickLabelMode = 'auto';
ax.XAxis.Visible = 'on';
ax.XTick = linspace(axlim(1), axlim(2), 11);
hold(ax,'on');

% Force text overlay
txtForce = text(ax, 0.02, 0.98, '', ...
    'Units','normalized','HorizontalAlignment','left', ...
    'VerticalAlignment','top','FontName','Consolas', ...
    'FontSize',11,'BackgroundColor','w','Margin',4);

%--- Draw rectangular wall ---%
if isfield(anim,'wall_x')
    wall_thickness = 0.1;
    wall_rect_x = [anim.wall_x, anim.wall_x + wall_thickness, ...
                   anim.wall_x + wall_thickness, anim.wall_x, anim.wall_x];
    wall_rect_y = [anim.wall_ymin, anim.wall_ymin, ...
                   anim.wall_ymax, anim.wall_ymax, anim.wall_ymin];
    fill(ax, wall_rect_x, wall_rect_y, [0.2 0.8 0.2], 'FaceAlpha', 0.5, ...
         'EdgeColor','y','LineWidth',2);
end

%--- Draw circular object ---%
if isfield(anim, 'circle_center') && isfield(anim, 'circle_radius')
    theta_draw = linspace(0, 2*pi, 50);
    circle_x = anim.circle_center(1) + anim.circle_radius * cos(theta_draw);
    circle_y = anim.circle_center(2) + anim.circle_radius * sin(theta_draw);
    fill(ax, circle_x, circle_y, [0.2 0.8 0.2], 'FaceAlpha', 0.5, ...
         'EdgeColor','y','LineWidth',2);
end

% Base, links, joints
plot(ax, 0, 0, 'ko', 'MarkerFaceColor','k', 'MarkerSize',6);
linkPlot  = plot(ax, [0, P1(1,1), P2(1,1)], [0, P1(1,2), P2(1,2)], ...
                 '-', 'LineWidth', 3);
jointPlot = plot(ax, [0, P1(1,1), P2(1,1)], [0, P1(1,2), P2(1,2)], ...
                 'o', 'MarkerFaceColor',[0.2 0.2 0.2], 'MarkerSize',6);

% Trace
if anim.showTrace
    tracePlot = plot(ax, P2(1,1), P2(1,2), '-', 'LineWidth', 1.5);
    traceX = zeros(numel(idx),1);
    traceY = zeros(numel(idx),1);
end

% Force arrows
forceScale = 0.002;
if isfield(anim,'F_mpc_x')
    arrowMPC  = quiver(ax, P2(1,1), P2(1,2), 0, 0, 'b',   'LineWidth',2, 'MaxHeadSize',0.8, 'AutoScale','off');
    arrowCalc = quiver(ax, P2(1,1), P2(1,2), 0, 0, 'r--', 'LineWidth',2, 'MaxHeadSize',0.8, 'AutoScale','off');
    legend(ax, [linkPlot, arrowMPC, arrowCalc], ...
           {'Robot','MPC Force','Calc Force'}, 'Location','northeast');
end

%--- Open VideoWriter ---%
if anim.record
    vWriter           = VideoWriter(anim.outputFile, anim.videoFormat);
    vWriter.FrameRate = anim.fps;
    if ismember(anim.videoFormat, {'MPEG-4','Motion JPEG AVI'})
        vWriter.Quality = anim.videoQuality;
    end
    open(vWriter);
    fprintf('[animateTwoLinkPlanar] Recording → %s.%s  (%d fps, %d frames)\n', ...
            anim.outputFile, vWriter.FileFormat, anim.fps, numel(idx));
end

%--- Animation loop ---%
tic;
for ii = 1:numel(idx)
    k = idx(ii);
    xdata = [0, P1(k,1), P2(k,1)];
    ydata = [0, P1(k,2), P2(k,2)];
    set(linkPlot,  'XData', xdata, 'YData', ydata);
    set(jointPlot, 'XData', xdata, 'YData', ydata);

    if isfield(anim,'F_mpc_x')
        set(arrowMPC,  'XData', P2(k,1), 'YData', P2(k,2), ...
                       'UData', anim.F_mpc_x(k)*forceScale, ...
                       'VData', anim.F_mpc_y(k)*forceScale);
        set(arrowCalc, 'XData', P2(k,1), 'YData', P2(k,2), ...
                       'UData', anim.F_calc_x(k)*forceScale, ...
                       'VData', anim.F_calc_y(k)*forceScale);
        if isgraphics(txtForce)
            txtForce.String = sprintf([ ...
                't = %.3f s\n' ...
                'MPC   F = [%8.2f, %8.2f] N\n' ...
                'Calc  F = [%8.2f, %8.2f] N\n' ...
                'Err   F = [%8.2f, %8.2f] N'], ...
                tRender(ii), ...
                anim.F_mpc_x(k), anim.F_mpc_y(k), ...
                anim.F_calc_x(k), anim.F_calc_y(k), ...
                anim.F_mpc_x(k)-anim.F_calc_x(k), ...
                anim.F_mpc_y(k)-anim.F_calc_y(k));
        end
    end

    if anim.showTrace
        traceX(ii) = P2(k,1);
        traceY(ii) = P2(k,2);
        switch lower(anim.traceMode)
            case 'persistent'
                set(tracePlot,'XData',traceX(1:ii),'YData',traceY(1:ii));
            case 'tail'
                i0 = max(1, ii - max(1, anim.traceLength));
                set(tracePlot,'XData',traceX(i0:ii),'YData',traceY(i0:ii));
        end
    end

    drawnow;

    if anim.record
        writeVideo(vWriter, getframe(fig));
    else
        if ii < numel(idx)
            targetTime = tRender(ii+1) - tRender(1);
            pause(max(0, targetTime - toc));
        end
    end
end

%--- Close VideoWriter ---%
if anim.record
    close(vWriter);
    fprintf('[animateTwoLinkPlanar] Saved: %s.%s\n', ...
            anim.outputFile, vWriter.FileFormat);
end
end
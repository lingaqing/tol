%PlotSNOPT/movie PlotSNOPT class movie plotting function.
%   movie([start = 0], [finish = 0]) plots the aircraft 3D trajectory and
%   state history animations
%
%   Example:
%
%       obj.movie;      %plots all legs
%       obj.movie(2);   %plots 2nd leg
%       obj.movie(2,4)  %plots legs 2-4
%
%   silvaw 05-28-15 - Original

function movie(self,varargin)
%*=+--+=#=+--      EA-DDDAS Trajectory Optimization Layer         --+=#=+--+=#*%
%          Copyright (C) 2015 Regents of the University of Colorado.           %
%                             All Rights Reserved.                             %
%                                                                              %
%    This program is free software: you can redistribute it and/or modify      %
%    it under the terms of the GNU General Public License version 2 as         %
%    published by the Free Software Foundation.                                %
%                                                                              %
%    This program is distributed in the hope that it will be useful,           %
%    but WITHOUT ANY WARRANTY; without even the implied warranty of            %
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             %
%    GNU General Public License for more details.                              %
%                                                                              %
%    You should have received a copy of the GNU General Public License         %
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.     %
%                                                                              %
%           Will Silva                                                         %
%           william.silva@colorado.edu                                         %
%                                                                              %
%*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*%
lasttime = 0;
if nargin == 1
    start = 1;
    finish = numel(self.legs);
elseif nargin == 2
    start = varargin{1};
    finish = varargin{1};
elseif nargin == 3
    start = varargin{1};
    finish = varargin{2};
else
    display('Too many args')
    return
end

plotstate = true;

%==================================================
% Initialize plotting parameters to generate figure
x = []; y = []; z = []; T =[];
t = 0;
for i = start:finish
    current_obj = self.legs(i);
    x = [x, current_obj.trajectory.x+current_obj.args.stormx];
    y = [y, current_obj.trajectory.y+current_obj.args.stormy];
    z = [z, -current_obj.trajectory.z+current_obj.args.stormz];
    T = [T, current_obj.trajectory.T];
    t = current_obj.trajectory.time(end) + t;
end

%determine extremes across all legs
xmin = min(x) - 10;
xmax = max(x) + 10;
ymin = min(y) - 10;
ymax = max(y) + 10;
zmin = min(z);
zmax = max(z)+10;
Tmax = max(T);
tvec = linspace(0,t,100);

%===================================================
% Generate figure(s)
trajfig = figure('Resize','off','name','Trajectory', 'units','pixels','Position',[0 0 800 600]);
%set(gcf,'GraphicsSmoothing','off')
suptitle('Trajectory');
grid on
axis equal
xlim([ymin ymax]);
ylim([xmin xmax]);
zlim([zmin zmax]);
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
azimuth = -45; elevation = 30;
view(azimuth,elevation)
axis vis3d
shadow = animatedline('Color','k');

if plotstate == true
    statefig = figure('Resize','off','name','State History', 'units','pixels','Position',[900 0 800 600]);
    ylabels = {'$V_a (m/s)$','$\gamma (deg)$','$\chi$ (deg)','$\phi (deg)$','$C_L$','$\dot{\phi} (deg/s)$','$T (N)$','$\Delta t (s)$'};
    suptitle('State History');
    ax = zeros(1,8);
    for i = 1:8
        ax(i) = subplot(4,2,i);
        h(i) = animatedline('Color','k');
        grid on
        %axis equal
        xlim([0 t]); %time
        ylabel(ylabels(i),'interpreter','latex');
        if i == 7 || i == 8
            xlabel('time (s)','interpreter','latex')
        end
    end
end

%====================================================
% Plot static objects only ONCE.

set(0,'currentfigure',trajfig);
% Color bar
caxis([0 Tmax])
p2bar = colorbar;
ylabel(p2bar,'Thrust (N)');

% Linear Boundary Layer
% Vref = 1;
% href = 10; hold on
% plot3([xmin xmin],[ymax (ymax-zmax*Vref/href)],[0 zmax],'b','LineWidth',1.3);
% plot3([xmin xmin],[ymax (ymax-zmax*Vref/href)],[zmax zmax],'b','LineWidth',1.3);
% plot3([xmin xmin],[ymax (ymax-zmax*Vref/href*.75)],[zmax*.75 zmax*.75],'b','LineWidth',1.3);
% plot3([xmin xmin],[ymax (ymax-zmax*Vref/href*0.5)],[zmax*.5 zmax*.5],'b','LineWidth',1.3);
% plot3([xmin xmin],[ymax (ymax-zmax*Vref/href*.25)],[zmax*.25 zmax*.25],'b','LineWidth',1.3);
% hold off

if plotstate == true
    set(0,'currentfigure',statefig);
end
    
        
%====================================================
% Create video writer object/settings
writerObj = VideoWriter('trajectory.avi');
open(writerObj);

if plotstate == true
    writerObj1 = VideoWriter('statehistory.avi');
    open(writerObj1);
end

frame_scale = 10; %plot aircraft, trajectory, shadow every 10th
line_scale = 100; %plot shadow line every 100th 

%====================================================
% Roll through each leg
for i = start:finish
    current_obj = self.legs(i);
    t       = current_obj.trajectory.time + lasttime; lasttime = t(end);
    x       = current_obj.trajectory.x + current_obj.args.stormx;
    y       = current_obj.trajectory.y + current_obj.args.stormy;
    z       = -current_obj.trajectory.z + current_obj.args.stormz;
    Va      = current_obj.trajectory.Va;
    gamma   = current_obj.trajectory.gam;
    chi     = current_obj.trajectory.chi;
    phi     = current_obj.trajectory.phi;
    CL      = current_obj.trajectory.CL;
    dphi  = current_obj.trajectory.dphi;
    dCL   = current_obj.trajectory.dCL;
    T       = current_obj.trajectory.T;
    dt      = current_obj.dt;

    min_limits = [current_obj.aircraft.Vamin, -current_obj.aircraft.gammamax*180/pi, 0, -current_obj.aircraft.phimax*180/pi,current_obj.aircraft.CLmin...
                 -current_obj.aircraft.dphimax*180/pi, current_obj.aircraft.Tmin, current_obj.limits.dtmin];
    max_limits = [current_obj.aircraft.Vamax, current_obj.aircraft.gammamax*180/pi, 360, current_obj.aircraft.phimax*180/pi,current_obj.aircraft.CLmax...
                 current_obj.aircraft.dphimax*180/pi, current_obj.aircraft.Tmax, current_obj.limits.dtmax];

    %load goal info to plot a loiter region if problem is loiter with kp
    if strcmp(current_obj.problem,'S10') && current_obj.gains.kp ~= 0
        goal.x = current_obj.args.xg;
        goal.y = current_obj.args.yg;
        goal.z = current_obj.args.zg;
        goal.radius = current_obj.args.rd;
    end

    Np = 1000;
    tb = linspace(t(1),t(end),Np);     % Create new time vector
    xb = interp1(t,x,tb);
    yb = interp1(t,y,tb);
    zb = interp1(t,z,tb);
    Vab = interp1(t,Va,tb);
    gammab = interp1(t,gamma,tb);
    chib = interp1(t,chi,tb);
    phib = interp1(t,phi,tb);
    CLb = interp1(t,CL,tb);
    dphib = interp1(t,dphi,tb);
    dCLb = interp1(t,dCL,tb);
    Tb = interp1(t,T,tb);
    dtb = ones(1,length(tb))*dt;
    
    if plotstate == true
        plottedstates = [Vab',[gammab.*180/pi]',[chib.*180/pi]',[phib.*180/pi]',CLb',[dphib.*180/pi]',Tb',dtb'];

        %==========================================================================
        % Plot Static elements (per leg)
        for i = 1:8
            subplot(4,2,i);
            hold on
            plot(tb,ones(1,length(tb))*min_limits(i),'--r')
            plot(tb,ones(1,length(tb))*max_limits(i),'--r')
            hold off
        end
    end
    
    %==========================================================================
    % Plot Dynamic elements
    plot_frame = 1; %initialize
    plot_line = 1;  %initialize

    for ii = 1:(length(tb))
        if ii == plot_frame
            %----------
            % Plot ac, trajectory, shadowline, and shadow
            set(0,'currentfigure',trajfig); hold on
            if plot_frame < length(tb) && ii ~= 1
                  delete(handles);
            end
            handles(1) = self.drawAircraft_Solid_alt([xb(ii) yb(ii) -zb(ii) phib(ii) gammab(ii) chib(ii)], trajfig);
            handles(2) = self.cline(yb(1:ii),xb(1:ii),zb(1:ii),Tb(1:ii));
            set(handles(2),'linewidth',4);
            if ii == plot_line
                %shadow line
                plot3([yb(ii) yb(ii)], [xb(ii) xb(ii)], [zb(ii) zmin],'k'); %not part of handles bc won't be deleted every time
                plot_line = plot_line + line_scale;
            end
            addpoints(shadow,yb(ii),xb(ii),zmin);
            %----------
            %Write frame to vid object
            writeVideo(writerObj, getframe(trajfig));
            
            if plotstate == true
                set(0,'currentfigure',statefig); hold on
                %----------
                % Plot states
                if plot_frame < length(tb) && ii ~= 1
                    %delete(statehandles);
                end
                
                for iii = 1:8
                    subplot(ax(i)); hold on;
                    addpoints(h(iii),tb(ii),plottedstates(ii,iii));
                    %statehandles(iii) = plot(tb(1:ii),plottedstates(1:ii,iii),'b','LineWidth',2);
                    hold off
                end
                %----------
                %Write frame to vid object
                writeVideo(writerObj1, getframe(statefig));
            end
            plot_frame = plot_frame + frame_scale;
        end
    end
    delete(handles(1));
end

        %360 degree spin while flying
        %azimuth = azimuth + 360/length(tb);
        %view(azimuth,elevation)

%360 degree spin after flying
% spinframes = 100;
% for ii = 1:spinframes;
%     azimuth = azimuth + 360/spinframes;
%     view(azimuth,elevation)
%     Mov(frame_ind) = getframe(h1);
%     frame_ind = frame_ind + 1;
% end

close(writerObj);
if plotstate == true
close(writerObj1);
end
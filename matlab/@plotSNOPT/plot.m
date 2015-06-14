%PlotSNOPT/plot PlotSNOPT class plotting function.
%   [h1,h2] = plot([start = 0], [finish = 0]) plots the aircraft 3D trajectory and state history
%
%   Example:
%
%       obj.plot;      %plots all legs
%       obj.plot(2);   %plots 2nd leg
%       obj.plot(2,4)  %plots legs 2-4
%
%   silvaw 04-30-15 - Original
%   silvaw 05-19-15 - Added feature to plot multiple legs

function [trajplot_enu, statehist] = plot(self,varargin)
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

trajplot_ned = figure('Position',[0 0 800 600]);
%trajplot_enu = figure('Position',[0 0 800 600]);
statehist = figure('Position',[1000 0 800 600]);
windplot = figure('Position',[0 0 800 600]);

if start ~= 1
    lasttime = self.legs(start-1).trajectory.time(end);
else
    lasttime = 0;
end
for i = start:finish
    current_obj = self.legs(i);

%% Extract variables from trajectory structure
t       = current_obj.trajectory.time + lasttime; lasttime = t(end);
x       = current_obj.trajectory.x;
y       = current_obj.trajectory.y;
z       = current_obj.trajectory.z;
Va      = current_obj.trajectory.Va;
gamma   = current_obj.trajectory.gam;
chi     = current_obj.trajectory.chi;
phi     = current_obj.trajectory.phi;
phidot  = current_obj.trajectory.dphi;
CL      = current_obj.trajectory.CL;
T       = current_obj.trajectory.T;
dt     = current_obj.dt;

pos_east = current_obj.args.east;
pos_north = current_obj.args.north;
pos_up = current_obj.args.up;
goal.x = current_obj.args.xg;
goal.y = current_obj.args.yg;
goal.z = current_obj.args.zg;
goal.radius = current_obj.args.rd;

Vamin = current_obj.aircraft.Vamin;
Vamax = current_obj.aircraft.Vamax;
gammamax = current_obj.aircraft.gammamax;
phimax = current_obj.aircraft.phimax;
CLmin = current_obj.aircraft.CLmin;
CLmax  = current_obj.aircraft.CLmax;
dtmin = current_obj.limits.dtmin;
dtmax = current_obj.limits.dtmax;
dphimax = current_obj.aircraft.dphimax;
Tmin = current_obj.aircraft.Tmin;
Tmax = current_obj.aircraft.Tmax;

FinalCost = current_obj.FinalCost;
problem = current_obj.problem;

%% Plot 3D trajectory (NED)
figure(trajplot_ned), hold on
% 
%plot initial trajectory
% initial = csvread('../Ioutput.txt');
% plot3(initial(2:11:end),initial(3:11:end),initial(4:11:end))
% xi = initial(2:11:end);
% yi = initial(3:11:end);
% zi = initial(4:11:end);
% phii =  initial(8:11:end);
% gammai = initial(6:11:end);
% chii = initial(7:11:end);
% scale = 10;
% plot_me = 1;
% for ii = 1:length(xi)
%     if ii == plot_me
%         self.drawAircraft_Solid([ xi(ii) yi(ii) zi(ii) phii(ii) gammai(ii) chii(ii) ], trajplot_ned);
%         plot_me = plot_me + scale;
%     end
% end

%plot a colored trajectory based on T
h = self.cline(x+pos_north, y+pos_east, z+(-pos_up), T);
hc = colorbar; set(h,'linewidth',4);
ylabel(hc,'Thrust (N)');
xlabel('x (north)'); ylabel('y (east)'); zlabel('z (down)')
axis equal
set(gca,'ZDir','Reverse')
set(gca,'XDir','Reverse')
title('NORTH EAST DOWN')

% Plot aircraft on trajectory
scale = 10;
plot_me = 1;
for ii = 1:length(x)
    if ii == plot_me
        self.drawAircraft_Solid([x(ii)+pos_north y(ii)+pos_east z(ii)+(-pos_up) phi(ii) gamma(ii) chi(ii)], trajplot_ned);
        plot_me = plot_me + scale;
    end
end

%title([current_obj.problem,' FinalCost: ',num2str(current_obj.FinalCost)]);
view(3); axis equal; grid on;

%get rid of scientific notation axes
%set(findall(gcf,'type','text'),'FontSize',30,'fontWeight','Bold')
set(gca,'FontSize',20);

if strcmp(current_obj.problem,'problemLT') || strcmp(current_obj.problem,'problemLNT') || strcmp(current_obj.problem,'S10') && current_obj.gains.kp ~= 0
    goalt = 0:.1:2*pi;
    hold on
    plot3(pos_north+goal.x+goal.radius*cos(goalt),pos_east+goal.y+goal.radius*sin(goalt),(-pos_up)+goal.z*ones(size(goalt)),'--','LineWidth',2);
end

if 0  %manual flag if you're plotting against a fixed wind shear
if i == 1 %plots a wind shear
Vref = 2.5;
href = 10;
xmin = pos_north;
xmax = self.legs(3).args.north + max(self.legs(3).trajectory.north);
ymax = storm.east + max(y);
zmin = storm.up;
zmax = storm.up + max(z);
yoffset = 30;

%Plot gradient surface (didn't really like the look of it)
% p1 = patch([xmin, xmax, xmax, xmin, xmin],...
%     [ymax+yoffset, ymax+yoffset,(ymax+yoffset-max(z)*Vref/href), (ymax+yoffset-max(z)*Vref/href), ymax+yoffset],...
%     [zmin, zmin, zmax, zmax, zmin],'blue');
% 
% p1.FaceVertexAlphaData = [zmin, zmin, zmax, zmax, zmin]';
% p1.FaceAlpha = 'interp';

for i = 0:100:400
plot3([xmin+i,xmin+i],[ymax+yoffset,(ymax+yoffset-max(z)*Vref/href)],[zmin,zmax],'LineWidth',1.3,'Color','b');
plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href)],[zmax,zmax],'LineWidth',1.3,'Color','b');
plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href*.75)],[(max(z)*.75+storm.z), (max(z)*.75+storm.z)],'LineWidth',1.3,'Color','b');
plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href*.5)],[(max(z)*.5+storm.z), (max(z)*.5+storm.z)],'LineWidth',1.3,'Color','b');
plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href*.25)],[(max(z)*.25+storm.z), (max(z)*.25+storm.z)],'LineWidth',1.3,'Color','b');
end
end
end

% %% Plot 3D trajectory (ENU)
% figure(trajplot_enu)
% 
% %plot initial trajectory
% initial = csvread('../Ioutput.txt');
% xi = initial(2:11:end);
% yi = initial(3:11:end);
% zi = initial(4:11:end);
% phii =  initial(8:11:end);
% gammai = initial(6:11:end);
% chii = initial(7:11:end);
% plot3(yi,xi,-zi)
% scale = 10;
% plot_me = 1;
% for ii = 1:length(xi)
%     if ii == plot_me
%         self.drawAircraft_Solid_alt([ xi(ii) yi(ii) zi(ii) phii(ii) gammai(ii) chii(ii) ], trajplot_enu);
%         plot_me = plot_me + scale;
%     end
% end
% 
% %plot a colored trajectory based on T
% h = self.cline(y+storm.y,x+storm.x,-z+storm.z,T);
% hc = colorbar; set(h,'linewidth',4);
% ylabel(hc,'Thrust (N)');
% xlabel('East'); ylabel('North'); zlabel('Up')
% axis equal
% title('EAST NORTH UP')
% 
% % Plot aircraft on trajectory
% scale = 10;
% plot_me = 1;
% for ii = 1:length(x)
%     if ii == plot_me
%         self.drawAircraft_Solid_alt([x(ii)+storm.x y(ii)+storm.y z(ii)+(-storm.z) phi(ii) gamma(ii) chi(ii)], trajplot_enu);
%         plot_me = plot_me + scale;
%     end
% end
% 
% %title([current_obj.problem,' FinalCost: ',num2str(current_obj.FinalCost)]);
% view(3); axis equal; grid on;
% 
% %get rid of scientific notation axes
% %set(findall(gcf,'type','text'),'FontSize',30,'fontWeight','Bold')
% set(gca,'FontSize',20);
% 
% if strcmp(current_obj.problem,'problemLT') || strcmp(current_obj.problem,'problemLNT') || strcmp(current_obj.problem,'problemS10') && current_obj.gains.kp ~= 0
%     goalt = 0:.1:2*pi;
%     hold on
%     plot3(storm.x+goal.x+goal.radius*cos(goalt),storm.y+goal.y+goal.radius*sin(goalt),storm.z+goal.z*ones(size(goalt)),'--','LineWidth',2);
% end
% 
% if 0  %manual flag if you're plotting against a fixed wind shear
% if i == 1 %plots a wind shear
% Vref = 2.5;
% href = 10;
% xmin = storm.x;
% xmax = self.legs(3).args.stormx + max(self.legs(3).trajectory.x);
% ymax = storm.y + max(y);
% zmin = storm.z;
% zmax = storm.z + max(z);
% yoffset = 30;
% 
% %Plot gradient surface (didn't really like the look of it)
% % p1 = patch([xmin, xmax, xmax, xmin, xmin],...
% %     [ymax+yoffset, ymax+yoffset,(ymax+yoffset-max(z)*Vref/href), (ymax+yoffset-max(z)*Vref/href), ymax+yoffset],...
% %     [zmin, zmin, zmax, zmax, zmin],'blue');
% % 
% % p1.FaceVertexAlphaData = [zmin, zmin, zmax, zmax, zmin]';
% % p1.FaceAlpha = 'interp';
% 
% for i = 0:100:400
% plot3([xmin+i,xmin+i],[ymax+yoffset,(ymax+yoffset-max(z)*Vref/href)],[zmin,zmax],'LineWidth',1.3,'Color','b');
% plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href)],[zmax,zmax],'LineWidth',1.3,'Color','b');
% plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href*.75)],[(max(z)*.75+storm.z), (max(z)*.75+storm.z)],'LineWidth',1.3,'Color','b');
% plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href*.5)],[(max(z)*.5+storm.z), (max(z)*.5+storm.z)],'LineWidth',1.3,'Color','b');
% plot3([xmin+i xmin+i],[ymax+yoffset (ymax+yoffset-max(z)*Vref/href*.25)],[(max(z)*.25+storm.z), (max(z)*.25+storm.z)],'LineWidth',1.3,'Color','b');
% end
% end
% end
%% Plot states and inputs
figure(statehist)


%Va
subplot(5,2,1), hold on

plot(t,Va,'LineWidth',2)
ylabel('Va (m/s)')
hold on
plot(t,ones(1,length(t))*Vamin,'--r')
plot(t,ones(1,length(t))*Vamax,'--r')
hold off

%gamma
subplot(5,2,2), hold on
plot(t,gamma.*180/pi,'LineWidth',2)
ylabel('\gamma (deg)')
hold on
plot(t,-ones(1,length(t))*gammamax*180/pi,'--r','LineWidth',2)
plot(t,ones(1,length(t))*gammamax*180/pi,'--r','LineWidth',2)
hold off

%chi
subplot(5,2,3), hold on
plot(t,chi.*180/pi,'LineWidth',2)
ylabel('\chi (deg)')
hold on
plot(t,ones(1,length(t))*360,'--r')
hold off

%phi
subplot(5,2,4), hold on
plot(t,phi.*180/pi,'LineWidth',2)
ylabel('\phi (deg)')
hold on
plot(t,-ones(1,length(t))*phimax*180/pi,'--r','LineWidth',2)
plot(t,ones(1,length(t))*phimax*180/pi,'--r','LineWidth',2)
hold off

%CL
subplot(5,2,6), hold on
plot(t,CL,'LineWidth',2)
ylabel('CL')
hold on
plot(t,ones(1,length(t))*CLmin,'--r','LineWidth',2)
plot(t,ones(1,length(t))*CLmax,'--r','LineWidth',2)
hold off

%dt
subplot(5,2,8), hold on
plot(t,ones(1,length(t))*dt,'LineWidth',2)
xlabel('Time (s)')
ylabel('dt')
hold on
plot(t,ones(1,length(t))*dtmin,'--r','LineWidth',2)
plot(t,ones(1,length(t))*dtmax,'--r','LineWidth',2)
hold off

%phidot
subplot(5,2,5), hold on
plot(t,phidot.*180/pi,'LineWidth',2)
ylabel('dphi')
hold on
plot(t,-ones(1,length(t))*dphimax*180/pi,'--r','LineWidth',2)
plot(t,ones(1,length(t))*dphimax*180/pi,'--r','LineWidth',2)
hold off

%T
subplot(5,2,7), hold on
plot(t,T,'LineWidth',2)
xlabel('Time (s)')
ylabel('T')
hold on
plot(t,ones(1,length(t))*Tmin,'--r','LineWidth',2)
plot(t,ones(1,length(t))*Tmax,'--r','LineWidth',2)
hold on


for i=1:8
   axesHandles(i) = subplot(5,2,i);
end
set(axesHandles,'fontSize',20)


axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
%suptitle([problem,' FinalCost: ',num2str(FinalCost)])
%figure(trajplot)

%% Plot Wind History
% figure(windplot)
% 
% wind = csvread('../Woutput.txt');
% u = wind(:,1);
% v = wind(:,2);
% w = wind(:,3);
% 
% du_dx = wind(:,4);
% du_dy = wind(:,5);
% du_dz  = wind(:,6);
% dv_dx = wind(:,7);
% dv_dy = wind(:,8);
% dv_dz  = wind(:,9);
% dw_dx = wind(:,10);
% dw_dy = wind(:,11);
% dw_dz  = wind(:,12);
% 
% %U (EAST)
% subplot(2,3,1), hold on, grid on
% plot(t,u,'LineWidth',2)
% ylabel('m/s')
% title('U (EAST)')
% hold off
% 
% %V (NORTH)
% subplot(2,3,2), hold on, grid on
% plot(t,v,'LineWidth',2)
% title('V (NORTH)')
% hold off
% 
% %W (UP)
% subplot(2,3,3), hold on, grid on
% plot(t,w,'LineWidth',2)
% title('W (UP)')
% hold off
% 
% %DU_D*
% subplot(2,3,4), hold on, grid on
% plot(t,du_dx,'LineWidth',2)
% plot(t,du_dy,'LineWidth',2)
% plot(t,du_dz,'LineWidth',2)
% xlabel('time (s)')
% ylabel('s^{-1}')
% title('\nablaU')
% legend('dU/dx','dU/dy','dU/dz')
% hold off
% 
% 
% %DV_D*
% subplot(2,3,5), hold on, grid on
% plot(t,dv_dx,'LineWidth',2)
% plot(t,dv_dy,'LineWidth',2)
% plot(t,dv_dz,'LineWidth',2)
% xlabel('time (s)')
% title('\nablaV')
% legend('dV/dx','dV/dy','dV/dz')
% hold off
% 
% 
% %DW_D*
% subplot(2,3,6), hold on, grid on
% plot(t,dw_dx,'LineWidth',2)
% plot(t,dw_dy,'LineWidth',2)
% plot(t,dw_dz,'LineWidth',2)
% xlabel('time (s)')
% title('\nablaW')
% legend('dW/dx','dW/dy','dW/dz')
% hold off
end
    
end

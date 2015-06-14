%PlotSNOPT/plotstorm PlotSNOPT storm plotter.
%   plotstorm() plots the entire storm with rain contours and wind
%   streamslices. It also plots where the aircraft is in the storm and
%   provides a zoomed in perspective. Last, it plots the aircraft
%   trajectory in a visualized wind field.
%
%   Example:
%
%       obj.plotstorm;
%
%  silvaw 04-30-15

function [self] = plotstorm(self)
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

%% Plot storm and wind field
if isempty(self.simdata)
    try
        simdata = load('../../../storm.mat');
    catch error
        [load_name,load_path,~] = uigetfile({'*.mat'},'Please locate storm data file');
        simdata = load([load_path load_name]);
    end
    self.simdata = simdata.simdata;
end


%clear simdata

storm.x = self.legs(1).arguments.stormx;
storm.y = self.legs(1).arguments.stormy;
storm.z = self.legs(1).arguments.stormz;
goal.x = self.legs(1).arguments.xg;
goal.y = self.legs(1).arguments.yg;
goal.z = self.legs(1).arguments.zg;

xmin = -50;
xmax = 500;
ymin = -150;
ymax = 150;
zmin = 0;
zmax = 0;

xmin = xmin + storm.x;
xmax = xmax + storm.x;
ymin = ymin + storm.y;
ymax = ymax + storm.y;
zmin = zmin + storm.z;
zmax = zmax + storm.z;

traj = self.plot();

figure(traj)
hold on

[X,Y,Z] = meshgrid(xmin:15:xmax,ymin:15:ymax,zmin:10:zmax);

[w,Jw] = arrayfun(@self.interpolatewind,X,Y,Z,'UniformOutput',false);
U = cellfun(@(v) v(1), w);
V = cellfun(@(v) v(2), w);
W = cellfun(@(v) v(3), w);

%windplot = figure('units','normalized','outerposition',[0 0 1 1]);
q = quiver3(X,Y,Z,U,V,W);
q.Color = 'blue';
xlabel('X'); ylabel('Y'); zlabel('Z'); 
grid on
axis equal

%get rid of scientific notation axes
% curtick = get(gca, 'XTick');
% set(gca, 'XTickLabel', cellstr(num2str(curtick(:))));
% curtick = get(gca, 'YTick');
% set(gca, 'YTickLabel', cellstr(num2str(curtick(:))));

%STORM PERSEPCTIVE
stormplot = figure('units','normalized','outerposition',[0 0 1 1]);
subplot(1,2,1)
contour(self.simdata.x0:self.simdata.xs:self.simdata.xf,self.simdata.y0:self.simdata.ys:self.simdata.yf,self.simdata.rain(:,:,1,3))
colormap(autumn)
hold on
streamslice(self.simdata.x0:self.simdata.xs:self.simdata.xf,self.simdata.y0:self.simdata.ys:self.simdata.yf,self.simdata.u(:,:,1,3)+self.simdata.frame.u,self.simdata.v(:,:,1,3)+self.simdata.frame.v)
hold on
plot(storm.x,storm.y,'rx')
hold on
rectangle('Position',[storm.x-2000 storm.y-2000 4000 4000],'LineWidth',3)
grid on
axis equal
subplot(1,2,2)
[xlowi,xlow] = value2index(storm.x-2000,150);
[xuppi,xupp] = value2index(storm.x+2000,150);
[ylowi,ylow] = value2index(storm.y-2000,150);
[yuppi,yupp] = value2index(storm.y+2000,150);

[X,Y] = meshgrid(xlow:150:xupp,ylow:150:yupp);

streamslice(X,Y,(self.simdata.u(xlowi:xuppi,ylowi:yuppi,1,3)+self.simdata.frame.u)',(self.simdata.v(xlowi:xuppi,ylowi:yuppi,1,3)+self.simdata.frame.v)')
hold on
plot(storm.x,storm.y,'rx','markers',12)
grid on
axis equal
axis([storm.x-2000 storm.x+2000 storm.y-2000 storm.y+2000])
end

function [index,val] = value2index(value, step)
    if length(step) == 1
        index = round(value/step) + 1;
        if index == 0
            index = 1;
        end
        val = (index-1)*step;
    else
        index = 1;
        while(step(index) < value)
            index = index + 1;
        end
        val = step(index);
    end
    
end 
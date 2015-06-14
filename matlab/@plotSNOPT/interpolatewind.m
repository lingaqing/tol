%PlotSNOPT/interpolatewind PlotSNOPT wind interpolation routine
%   interpolatewind() interpolates the windfield for a given 3 dimensional
%   point. Can be used with mesh grid and arrayfun to generate an estimated
%   wind field. See it's use in the plotstorm() method for an example.
%
%  silvaw 04-30-15


function [wind,Jw] = interpolatewind(self,xs,ys,zs)
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
wind = [];
wind_gradient = [];

uref = 19;
vref = 2;

xspacing = 150;
yspacing = 150;
zspacing = self.simdata.z;

xindex = round(xs/xspacing);
yindex = round(ys/yspacing);
zindex = ones(length(zs),1);
for i =1:length(zs)
    while(zspacing(zindex(i)) < zs(i))
        zindex(i) = zindex(i) + 1;
    end
end

uref = 19;
vref = 2;

%bottom front left
u(:,1) = [self.simdata.u(xindex+1,yindex+1,zindex-1,3) + uref];

%top front left
u(2) = self.simdata.u(xindex+1,yindex+1,zindex,3) + uref;

%bottom back left
u(3) = self.simdata.u(xindex+1,yindex+2,zindex-1,3) + uref;

%top back left
u(4) = self.simdata.u(xindex+1,yindex+2,zindex,3) + uref;

%bottom front right
u(5) = self.simdata.u(xindex+2,yindex+1,zindex-1,3) + uref;

%top front right
u(6) = self.simdata.u(xindex+2,yindex+1,zindex,3) + uref;

%bottom back right
u(7) = self.simdata.u(xindex+2,yindex+2,zindex-1,3) + uref;

%top back right
u(8) = self.simdata.u(xindex+2,yindex+2,zindex,3) + uref;


%bottom front left
v(1) = self.simdata.v(xindex+1,yindex+1,zindex-1,3) + vref;

%top front left
v(2) = self.simdata.v(xindex+1,yindex+1,zindex,3) + vref;

%bottom back left
v(3) = self.simdata.v(xindex+1,yindex+2,zindex-1,3) + vref;

%top back left
v(4) = self.simdata.v(xindex+1,yindex+2,zindex,3) + vref;

%bottom front right
v(5) = self.simdata.v(xindex+2,yindex+1,zindex-1,3) + vref;

%top front right
v(6) = self.simdata.v(xindex+2,yindex+1,zindex,3) + vref;

%bottom back right
v(7) = self.simdata.v(xindex+2,yindex+2,zindex-1,3) + vref;

%top back right
v(8) = self.simdata.v(xindex+2,yindex+2,zindex,3) + vref;


%bottom front left
w(1) = self.simdata.w(xindex+1,yindex+1,zindex-1,3);

%top front left
w(2) = self.simdata.w(xindex+1,yindex+1,zindex,3);

%bottom back left
w(3) = self.simdata.w(xindex+1,yindex+2,zindex-1,3);

%top back left
w(4) = self.simdata.w(xindex+1,yindex+2,zindex,3);

%bottom front right
w(5) = self.simdata.w(xindex+2,yindex+1,zindex-1,3);

%top front right
w(6) = self.simdata.w(xindex+2,yindex+1,zindex,3);

%bottom back right
w(7) = self.simdata.w(xindex+2,yindex+2,zindex-1,3);

%top back right
w(8) = self.simdata.w(xindex+2,yindex+2,zindex,3);

xrel = (xs-(xindex*xspacing));
yrel = (ys-(yindex*yspacing));
zrel = (zs-(zspacing(zindex-1)));
dx = xspacing;
dy = yspacing;
dz = zspacing(zindex) - zspacing(zindex-1);
zeta = xrel/dx;
eta = yrel/dy;
mu = zrel/dz;

N(1) = (1-zeta)*(1-eta)*(1-mu);
N(5) = zeta*(1-eta)*(1-mu);
N(3) = (1-zeta)*eta*(1-mu);
N(7) = zeta*eta*(1-mu);
N(2) = (1-zeta)*(1-eta)*mu;
N(6) = zeta*(1-eta)*mu;
N(4) = (1-zeta)*eta*mu;
N(8) = zeta*eta*mu;

est_u = 0;
est_v = 0;
est_w = 0;

for i = 1:8
    est_u = est_u + N(i)*u(i);
    est_v = est_v + N(i)*v(i);
    est_w = est_w + N(i)*w(i);
end

wind = [wind; est_u est_v est_w];


NwrtX(1) = -((yrel/dy - 1)*(zrel/dz - 1))/dx;
NwrtX(5) = ((yrel/dy - 1)*(zrel/dz - 1))/dx;
NwrtX(3) = (yrel*(zrel/dz - 1))/(dx*dy); 
NwrtX(7) = -(yrel*(zrel/dz - 1))/(dx*dy);
NwrtX(2) = (zrel*(yrel/dy - 1))/(dx*dz);
NwrtX(6) = -(zrel*(yrel/dy - 1))/(dx*dz);  
NwrtX(4) = -(yrel*zrel)/(dx*dy*dz);
NwrtX(8) = (yrel*zrel)/(dx*dy*dz);


NwrtY(1) = -((xrel/dx - 1.0)*(zrel/dz - 1.0))/dy;
NwrtY(3) = (xrel*(zrel/dz - 1.0))/(dx*dy);
NwrtY(5) = ((xrel/dx - 1.0)*(zrel/dz - 1.0))/dy;
NwrtY(7) = -(xrel*(zrel/dz - 1.0))/(dx*dy);
NwrtY(2) = (zrel*(xrel/dx - 1.0))/(dy*dz);
NwrtY(6) = -(xrel*zrel)/(dx*dy*dz);
NwrtY(4) = -(zrel*(xrel/dx - 1.0))/(dy*dz);
NwrtY(8) = (xrel*zrel)/(dx*dy*dz);


NwrtZ(1) = -((xrel/dx - 1.0)*(yrel/dy - 1.0))/dz;
NwrtZ(3) = (xrel*(yrel/dy - 1.0))/(dx*dz);  
NwrtZ(5) = (yrel*(xrel/dx - 1.0))/(dy*dz);  
NwrtZ(7) = -(xrel*yrel)/(dx*dy*dz);   
NwrtZ(2) = ((xrel/dx - 1.0)*(yrel/dy - 1.0))/dz; 
NwrtZ(6) = -(xrel*(yrel/dy - 1.0))/(dx*dz);
NwrtZ(4) = -(yrel*(xrel/dx - 1.0))/(dy*dz); 
NwrtZ(8) = (xrel*yrel)/(dx*dy*dz);  

du_dx = 0;
du_dy = 0;
du_dz = 0;
dv_dx = 0;
dv_dy = 0;
dv_dz = 0;
dw_dx = 0;
dw_dy = 0 ;
dw_dz = 0;

for i = 1:8
    du_dx = du_dx + NwrtX(i)*u(i);
    du_dy = du_dy + NwrtY(i)*u(i);
    du_dz = du_dz + NwrtZ(i)*u(i);
    dv_dx = dv_dx + NwrtX(i)*v(i);
    dv_dy = dv_dy + NwrtY(i)*v(i);
    dv_dz = dv_dz + NwrtZ(i)*v(i);
    dw_dx = dw_dx + NwrtX(i)*w(i);
    dw_dy = dw_dy + NwrtY(i)*w(i);
    dw_dz = dw_dz + NwrtZ(i)*w(i);
end

Jw = [wind_gradient; [du_dx du_dy du_dz dv_dx dv_dy dv_dz dw_dx dw_dy dw_dz]];























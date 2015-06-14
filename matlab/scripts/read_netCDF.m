clear all
close all
clc
ncpath = '/home/silva/workspace/flightplan2/synthcress_1915.nc';
%ncdisp(ncpath, 'U');
u = ncread(ncpath,'U');
v = ncread(ncpath,'V');
w = ncread(ncpath,'W');
x = ncread(ncpath,'x');
y = ncread(ncpath,'y');
z = ncread(ncpath,'z');
i_lat = ncread(ncpath,'grid_latitude');
i_lon = ncread(ncpath,'grid_longitude');

nu = u ~= -32768;
nv = u ~= -32768;

newu = nu .* u;
newv = nv .* v;

quiver(x,y,newu(:,:,1)',newv(:,:,1)')

w_ns = 1000*input('Enter distance N/S (km): ');
w_ew = 1000*input('Enter distance E/W (km): ');

re = 6378137;

%increment of latitude
d_lat = w_ns*180/(pi*re);

%waypoint latitude
w_lat = i_lat + d_lat;

%correct for longitude lines getting closer together near poles
re_c = re*cosd(abs(i_lat));

d_lon = w_ew*180/(pi*re_c);

w_lon = i_lon + d_lon;

display(w_lat);
display(w_lon);

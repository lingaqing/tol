%% Plot 3D trajectory


h1 = figure('units','normalized','outerposition',[0 0 1 1]);

for i = 1:8
load(['./plot_SNOPT/StoredTraj/',num2str(i),'.mat'])

t       = results.data.time;
x       = results.data.x;
y       = results.data.y;
z       = -results.data.z;
Va      = results.data.Va;
gamma   = results.data.gam;
chi     = results.data.chi;
phi     = results.data.phi;
phidot  = results.data.dphi;
CL      = results.data.CL;
T       = results.data.T;
dtk     = results.data.time(2);

%should make these dynamic eventually (should be stored in results struct)


storm.x = results.args.stormx;
storm.y = results.args.stormy;
storm.z = results.args.stormz;
goal.x = results.args.xg;
goal.y = results.args.yg;
goal.z = results.args.zg;
goal.radius = results.args.rd;


%plot a colored trajectory based on T
h = cline(x+storm.x,y+storm.y,z+storm.z,T);
colorbar;
set(h,'linewidth',4)
xlabel('X')
ylabel('Y')
zlabel('Z')

axis equal
grid on

results.parameters.xg = 0;
results.parameters.yg = 0;

%----------
% Plot aircraft on trajectory
scale = 10;
plot_me = 1;
for ii = 1:length(x)
    if ii == plot_me
        drawAircraft_Solid([x(ii)+storm.x y(ii)+storm.y z(ii)+storm.z phi(ii) gamma(ii) chi(ii)], h1)
        plot_me = plot_me + scale;
    end
end
hold on
end

hold on
plot_storm(results)
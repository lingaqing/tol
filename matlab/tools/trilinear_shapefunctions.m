syms zeta eta mu xrel yrel zrel dx dy dz 

zeta = xrel/dx;
eta = yrel/dy;
mu = zrel/dz;

N0 = (1-zeta)*(1-eta)*(1-mu);
N1 = zeta*(1-eta)*(1-mu);
N2 = (1-zeta)*eta*(1-mu);
N3 = zeta*eta*(1-mu);
N4 = (1-zeta)*(1-eta)*mu;
N5 = zeta*(1-eta)*mu;
N6 = (1-zeta)*eta*mu;
N7 = zeta*eta*mu;

diff(N2,xrel)

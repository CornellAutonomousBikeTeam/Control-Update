function[results]  = findangle(p, v0, lag1, lag2, loc, results)
%Finds maximal angle and related controller values for linear and nonlinear
%controllers for a given set of parameters. 
x0 = 0;
y0 = 0;
phi0=0.01; 
delta0 = 0;
phi_dot0= 0;
Km  = [0,0,0];
a= 0; 
visual = 0; 
psi0 = 0;
K = [11.1, 2.4, -6.6];
[K1,K2,K3]= findLQR(p,v0, lag1,lag2);
[nav, max_val, t]  = findmax(p, v0, lag1, lag2, loc, K,x0, y0, delta0,phi0, phi_dot0, psi0,0, a,Km, visual);
 results(1,1:8)  = [0,0,t, max_val, nav, 11.1, 2.4,-6.6]; 
 [nav, max_val, t]  = findmax(p, v0, lag1, lag2, loc, K1,x0, y0, delta0,phi0, phi_dot0, psi0, 0, a,Km, visual);
 results(2,1:8)  = [1,0,t, max_val, nav,K1(1), K1(2), K1(3)];
 [nav, max_val, t]  = findmax(p, v0, lag1, lag2, loc, K2,x0, y0, delta0,phi0, phi_dot0, psi0, 0, a,Km, visual);
 results(3,1:8)  = [1,0,t, max_val, nav,K2(1), K2(2), K2(3)]; 
  [nav, max_val, t]  = findmax(p, v0, lag1, lag2, loc, K3,x0, y0, delta0,phi0, phi_dot0, psi0, 0, a,Km, visual);
 results(4,1:8)  = [1,0,t, max_val, nav,K3(1), K3(2), K3(3)]; 
 
for a  = linspace(0,6,7)
    [KB, KM]  = find_nonlinear(p, v0, 0,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, 1, a, 0, 1000, 9);
    
[nav, max_val, t]  = findmax(p, v0, lag1, lag2, loc, KB,x0, y0, delta0,phi0, phi_dot0, psi0, 1, a,KM, visual);
results(5+a,1:11)  = [1,a,t, max_val, nav,KB(1), KB(2), KB(3), KM(1), KM(2), KM(3)];
end
 
end 
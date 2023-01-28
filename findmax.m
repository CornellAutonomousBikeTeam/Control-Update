function[nav, max_val, t]  = findmax(p, v0, lag1, lag2, loc, K,x0, y0, delta0,phi0, phi_dot0, psi0, nonlinear, a,Km, visual)
%Locates the largest angle at which point balances for a time period of loc
deltad = zeros(1,400+loc); 
t = linspace(0, pi/2,50);
min = 0;
for t=t
    deltad(1,201:200+loc) = ones(1, loc)*t;
    nav = testSteerOffset(p, K, v0, deltad,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, 0);
    if t == 0 
        min = nav;
    end
    if nav-min>2*loc*t
        fprintf("nav: %d max : %d", nav, t); 
        display(nav);
        display(t);
        break
    end 
end

 fprintf("nav: %d max : %d", nav, t); 
 t = max(0, t - 0.05);
 deltad(1,201:200+loc) = ones(1, loc)*t;
[nav, max_val] = testSteerAlternate(p, K, v0, deltad,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, visual);
end 
function[results]  = bestperformance(p, v0, lag1, lag2, loc)
%Attempts to find the controller that can perform at the highest angle for
%each type/power of controller, saves these states in TRes.

x0 = 0;
y0 = 0;
phi0=0.01; 
delta0 = 0;
phi_dot0= 0;
nonlinear = 0;
Km  = [0,0,0];
a= 0;  
psi0 = 0;
    TRes = zeros(11,11);
    for i = 1:5
        
        temp  = findangle(p,3, 0,0,200, zeros(11,11));
        for  j = 1:11
            if(temp(j,4) > TRes(j,4))
            TRes(j, 1:11)  = temp(j, 1:11);
            end 
        end 
    end   
    for i = 1:4
        deltad = zeros(1,400 + loc);
        deltad(1,201:200+loc) = ones(1,loc)*TRes(i,3);
        K  =[TRes(i,6), TRes(i,7), TRes(i,8)];
        testSteerAlternate(p, K, v0, deltad,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, 1);

    end
    for i = 5:11
        deltad = zeros(1,400 + loc);
        deltad(1,201:200+loc) = ones(1,loc)*TRes(i,3);
        K  =[TRes(i,6), TRes(i,7), TRes(i,8)];
        Km = [TRes(i,9), TRes(i,10), TRes(i,11)];
        testSteerAlternate(p, K, v0, deltad,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, 1);
    end 
    results =TRes;
end


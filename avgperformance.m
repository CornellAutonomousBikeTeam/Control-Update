function[results]  = avgperformance(p, v0, lag1, lag2, loc)
%Finds and saves the average controller value and maximal angle achieved
%across several runs of finding controllers of each category, to account
%for randomness.
x0 = 0;
y0 = 0;
phi0=0.01; 
delta0 = 0;
phi_dot0= 0;
nonlinear = 0;
Km  = [0,0,0];
a= 0;  
psi0 = 0;
    TAvg = zeros(11,11);
    for i = 1:5
        TAvg = TAvg + findangle(p,3, 0,0,200, zeros(11,11));
    end 
    results = TAvg *.2;
    deltad = zeros(1,400 + loc);
    for i = 1:4
        deltad = zeros(1,400 + loc);
        deltad(1,201:200+loc) = ones(1,loc)*TAvg(i,3);
        K  =[TAvg(i,6), TAvg(i,7), TAvg(i,8)];
        testSteerAlternate(p, K, v0, deltad,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, 0);

    end
    for i = 5:11
        deltad = zeros(1,400 + loc);
        deltad(1,201:200+loc) = ones(1,loc)*TAvg(i,3);
        K  =[TAvg(i,6), TAvg(i,7), TAvg(i,8)];
        Km = [TAvg(i,9), TAvg(i,10), TAvg(i,11)];
        testSteerAlternate(p, K, v0, deltad,x0, y0, delta0,phi0, phi_dot0, psi0,lag1, lag2, nonlinear, a,Km, 0);
    end 
end
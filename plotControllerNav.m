function [] = plotControllerNav(x0,y0,v0,delta0,phi0, ...
                phi_dot0,psi0,p, K, xdes, ydes, lag1,lag2, numTimeSteps,  graph, maxangle, distscale)
%plots controller performance for given parameters with directed steer
%commands.
            arguments 
                x0  
                y0   
                v0   
                delta0 
                phi0
                phi_dot0 
                psi0
                p
                K   
                xdes
                ydes
                lag1   = 0
                lag2  = 0 
                numTimeSteps   = 200
                graph  = 0
                maxangle = pi/12
                distscale = 0.3
            end 
 numtime = numTimeSteps;
    [~, offset0] = runBicycleTestNav(x0,y0,v0,delta0,phi0, ...
                phi_dot0,psi0,p, K, xdes, ydes, lag1,lag2, numTimeSteps,  graph, maxangle, distscale);
 

    times = offset0(1:numtime,1);
    xs = offset0(1:numtime,2); 
    xdesx = offset0(1:numtime, 9);
     ys = offset0(1:numtime,3); 
    ydesy = offset0(1:numtime, 10);
    phi = offset0(1:numtime,4);
    delta = offset0(1:numtime,6);
    phidot = offset0(1:numtime,7);
    deltadot = diff(delta)./diff(times);
        
    subplot(2,3,1); 
    
    subplot(2,3,1)
    hold on
    plot(times,phi);
    title('lean vs. time');
    xlabel('time (s)');
    ylabel('phi');
    subplot(2,3,2)
    hold on
    plot(times,phidot);
    title('lean rate vs. time');
    xlabel('time (s)');
    ylabel('phi-dot');
    subplot(2,3,3)
    hold on
    plot(times,delta);
    title('steer vs. time');
    xlabel('time (s)');
    ylabel('delta'); 
    subplot(2,3,4)
    hold on
    plot(times(1:end-1),deltadot);
    title('steer rate vs. time');
    xlabel('time (s)');
    ylabel('deltadot');
    subplot(2,3,5)
    hold on
    plot(times,xs);
    plot(times,xdesx);
    title('x vs. time');
    xlabel('time (s)');
    ylabel('x'); 
    subplot(2,3,6)
    hold on
    plot(times,ys);
    plot(times,ydesy);
    title('y vs. time');
    xlabel('time (s)');
    ylabel('y'); 
    hold off
end 
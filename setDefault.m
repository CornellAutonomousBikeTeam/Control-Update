function g = setDefault
%Sets the default parameters in terminal to standards in the event that
%they are omitted or left as arguments. Adjust as necessary.
    global p
    p.g = 9.81;
    p.b = 0.33;
    p.h = 0.56;
    p.l = 1.02;
    p.pause = 0;
    p.s = 0;
    g = p;
end
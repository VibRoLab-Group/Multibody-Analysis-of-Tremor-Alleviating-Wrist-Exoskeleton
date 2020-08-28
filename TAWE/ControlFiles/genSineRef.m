function [pos,vel,acc]=genSineRef(t,w,amp,phase)
    pos=sin(w*t+phase).*amp;
    vel=w.*cos(w*t+phase).*amp;
    acc=-(w.^2).*sin(w*t+phase).*amp;
end
trajData=zeros(6,length(tspan));
freqNum=41;
for ii=1:2
    trajAmp=ii^1.5*3*pi/180*[rand(freqNum,1)];
    trajOmega=2*pi*((2).^linspace(-2,0,freqNum)).';
    trajPhase=2*pi*(rand(freqNum,1)-0.5);
    trajData(ii,:)=trajAmp.'*sin(trajOmega*tspan+(trajPhase)*ones(1,length(tspan)));
    trajData(2+ii,:)=(trajOmega.*trajAmp).'*cos(trajOmega*tspan+(trajPhase)*ones(1,length(tspan)));
    trajData(4+ii,:)=-(trajOmega.*trajOmega.*trajAmp).'*sin(trajOmega*tspan+(trajPhase)*ones(1,length(tspan)));
end

figure(99)
plot(tspan,trajData(1:2,:))

save('./ControlFiles/GenTrajData.mat','trajData');
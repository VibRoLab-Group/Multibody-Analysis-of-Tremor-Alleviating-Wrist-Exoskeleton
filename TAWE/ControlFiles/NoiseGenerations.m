% trajData=zeros(2,length(tspan));
freqNum=17;
for ii=1:2
    trajAmp=[rand(freqNum,1)];
    trajOmega=2*pi*(linspace(4,8,freqNum)).';
    trajPhase=2*pi*(rand(freqNum,1)-0.5);
    trajData=trajAmp.'*sin(trajOmega*tspan+(trajPhase)*ones(1,length(tspan)));
    noiseData(ii).amp=trajAmp;
    noiseData(ii).omega=trajOmega;
    noiseData(ii).phase=trajPhase;
    noiseData(ii).data=trajData;
end

figure(99)
plot(tspan,noiseData(1).data,'r')
hold on
plot(tspan,noiseData(2).data,'g')
hold off

save('./ControlFiles/GenNoiseData.mat','noiseData');
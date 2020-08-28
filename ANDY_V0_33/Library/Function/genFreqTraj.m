function trajOut=genFreqTraj(inOrder,inTime,inAmp,inFreq)

freqNum=numel(inFreq);
tSpan=reshape(inTime,1,[]);
dataNum=numel(tSpan);
freqArray=2*reshape(inFreq,[],1)*pi;

trajOut=zeros(inOrder,dataNum);

    for ii=1:inOrder
        
        ampArray=inAmp.*(rand([freqNum,2])-0.5);
        
        traj=(ampArray(:,1).')*([sin(freqArray.*tSpan)])+(ampArray(:,2).')*([cos(freqArray.*tSpan)]);
        trajOut(ii,:)=traj;

    end

end
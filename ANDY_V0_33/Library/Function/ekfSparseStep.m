function [modelState]=ekfSparseStep(mode,modelFunc,modelState)
    if mode>3||mode<1
        error("mode 1 for joint estimation; mode 2 for summed-up sparse; mode 3 for sequential");
    end
    mode=floor(mode);
        
    pp=modelState.pp;
    xx=modelState.xx;
    uu=modelState.uu;
    
    xxDim=numel(xx);
    
    PPxxFull=modelState.PPxxFull;
    QQ=modelState.QQ;
    QQBase=modelState.QQBase;
    RR=modelState.RR;
    RRBase=modelState.RRBase;
    
    
    xxSparse=modelState.xxSparse;
    xxbbSparse=modelState.xxbbSparse;
    xxAlphaSparse=modelState.xxAlphaSparse;
    xxrrSparse=modelState.xxrrSparse;
    xxThreshSparse=modelState.xxThreshSparse;
    
    xxSparseDim=numel(xxSparse);
        
    QQSparse=modelState.QQSparse;
    RRSparse=modelState.RRSparse;
    
    
    xxHat=modelFunc.xxPlus(pp,xx,xxSparse,uu);
    xxHatFull=[xxHat;xxSparse];
    FxxFull=[modelFunc.Fxx(pp,xx,xxSparse,uu);
             zeros(xxSparseDim,xxDim),eye(xxSparseDim)];
    Fvv=modelFunc.Fvv(pp,xx,xxSparse,uu);
    
    PPxxHatFull=FxxFull*PPxxFull*FxxFull.';
    PPxxHatFull(1:xxDim,1:xxDim)=PPxxHatFull(1:xxDim,1:xxDim)+Fvv*QQ*Fvv.'+QQBase;
    PPxxHatFull((1:xxSparseDim)+xxDim,(1:xxSparseDim)+xxDim)=PPxxHatFull((1:xxSparseDim)+xxDim,(1:xxSparseDim)+xxDim)+QQSparse;
        
    if mode==1
        yyHat=modelFunc.yy(pp,xxHat,xxSparse,uu);
        [yySparseHat,HxxSparseVec]=sparseSmoothFunc(modelState.sparseFuncParam(1),xxSparse,xxbbSparse,xxAlphaSparse,xxrrSparse);
        yyDim=numel(yyHat);
        yySparseDim=xxSparseDim;
        HxxFull=[modelFunc.Hxx(pp,xxHat,xxSparse,uu);
                 zeros(yySparseDim,xxDim),diag(HxxSparseVec)];
        Hww=modelFunc.Hww(pp,xxHat,xxSparse,uu);

        PPyyHatFull=HxxFull*PPxxHatFull*HxxFull.';
        PPyyHatFull(1:yyDim,1:yyDim)=PPyyHatFull(1:yyDim,1:yyDim)+Hww*RR*Hww.'+RRBase;
        PPyyHatFull(1+yyDim:end,1+yyDim:end)=PPyyHatFull(1+yyDim:end,1+yyDim:end)+RRSparse;

        PPxyFull=PPxxHatFull*HxxFull.';

        GGFull=PPxyFull/PPyyHatFull;
        xxFull=xxHatFull-GGFull*[yyHat;yySparseHat];
        modelState.PPxxFull=PPxxHatFull - GGFull*PPyyHatFull*GGFull.';
   
    elseif mode==2
        yyHat=modelFunc.yy(pp,xxHat,xxSparse,uu);
        [yySparseHat,HxxSparseVec]=sparseSmoothFunc_Sum(modelState.sparseFuncParam(1),modelState.sparseFuncParam(2),modelState.sparseFuncParam(3),xxSparse,xxbbSparse,xxAlphaSparse,xxrrSparse);
        yyDim=numel(yyHat);
        yySparseDim=1;
        
        HxxFull=[modelFunc.Hxx(pp,xxHat,xxSparse,uu);
                 zeros(yySparseDim,xxDim),(HxxSparseVec)];
        Hww=modelFunc.Hww(pp,xxHat,xxSparse,uu);

        PPyyHatFull=HxxFull*PPxxHatFull*HxxFull.';
        PPyyHatFull(1:yyDim,1:yyDim)=PPyyHatFull(1:yyDim,1:yyDim)+Hww*RR*Hww.'+RRBase;
        PPyyHatFull(1+yyDim:end,1+yyDim:end)=PPyyHatFull(1+yyDim:end,1+yyDim:end)+RRSparse(1,1);

        PPxyFull=PPxxHatFull*HxxFull.';

        GGFull=PPxyFull/PPyyHatFull;
        xxFull=xxHatFull-GGFull*[yyHat;yySparseHat];
        modelState.PPxxFull=PPxxHatFull - GGFull*PPyyHatFull*GGFull.';
        
    elseif mode==3       
        yyHat=modelFunc.yy(pp,xxHat,xxSparse,uu);
        Hxx=modelFunc.Hxx(pp,xxHat,xxSparse,uu);
        Hww=modelFunc.Hww(pp,xxHat,xxSparse,uu);

        PPyyHatFull=Hxx*PPxxHatFull*Hxx.'+Hww*RR*Hww.'+RRBase;
        PPxyFull=PPxxHatFull*Hxx.';

        GGFull=PPxyFull/PPyyHatFull;
        xxFull=xxHatFull-GGFull*yyHat;
        modelState.PPxxFull=PPxxHatFull - GGFull*PPyyHatFull*GGFull.';      
        
        PPxxSparseHatVec=modelState.PPxxSparseVec+modelState.QQSparseVec;
        [yySparseHat,HxxSparseVec]=sparseSmoothFunc(modelState.sparseFuncParam(1),xxSparse,xxbbSparse,xxAlphaSparse,xxrrSparse);
        PPyySparseHatVec=HxxSparseVec.*PPxxSparseHatVec.*HxxSparseVec+modelState.RRSparseVec;
        PPxySparseVec=PPxxSparseHatVec.*HxxSparseVec;
        GGSparseVec=PPxySparseVec./PPyySparseHatVec;
        xxFull((1:xxSparseDim)+xxDim,1)=xxFull((1:xxSparseDim)+xxDim,1)-GGSparseVec.*yySparseHat;
        modelState.PPxxSparseVec = PPxxSparseHatVec - GGSparseVec.*PPyySparseHatVec.*GGSparseVec;
        
    end
    
    modelState.yyHat=yyHat;
    modelState.xx=modelFunc.xxPost(pp,xxFull(1:xxDim),xxFull((1:xxSparseDim)+xxDim),uu);
    modelState.xxSparse=sparseThreshholding(xxFull((1:xxSparseDim)+xxDim),xxThreshSparse);
    
    
end

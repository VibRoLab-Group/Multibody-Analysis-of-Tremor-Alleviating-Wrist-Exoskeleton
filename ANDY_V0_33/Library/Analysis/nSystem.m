classdef nSystem < jSys
    properties(SetAccess=protected)
        TimeVar;
        ScaleVar;
        LaplaceVar;
        Order;
        
        State;
        Param;
        Input;
        DelayState;
        
        FixedPoint;
        FixedPointCondition;
        
        MMSSolver;
        
        IsInit;
        
    end
    
    methods(Access=public)
        function obj=nSystem(inName,inTime,inScale,inLaplace)
            obj@jSys(inName);
            
            tic;
            
            obj.TimeVar=inTime;
            obj.ScaleVar=inScale;
            obj.LaplaceVar=inLaplace;
            obj.Order=0;
            addpath(genpath(pwd));
            
            obj.FixedPoint=[];
            obj.FixedPointCondition=[];
            
            obj.IsInit=false;
            
            obj.State=jDic(obj.tagStr('State'),'nState');
            obj.Param=jDic(obj.tagStr('Param'),'nParam');
            obj.Input=jDic(obj.tagStr('Input'),'nInput');
            obj.DelayState=jDic(obj.tagStr('DelayState'),'nDelay');
            
            obj.MMSSolver=jDic(obj.tagStr('MMSSolver'),'nMMS');
        end
        
        function discHandle=genParam(obj,inParamSet)
            discHandle=[];
            if isa(inParamSet,'cell')
                discSize=size(inParamSet);
                if discSize(2)==2
                    for ii=1:discSize(1)
                        curParam=nParam(inParamSet{ii,1},obj,inParamSet{ii,2});
                        eval(strcat('discHandle.',char(curParam),'=curParam;'));
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {(name1) (des1);(name2) (des2);...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {(name1) (des1);(name2) (des2);...}'));
            end
        end
        
        function inputHandle=genInput(obj,inInputSet)
            inputHandle=[];
            if isa(inInputSet,'cell')
                inputSize=size(inInputSet);
                if inputSize(2)==2
                    for ii=1:inputSize(1)
                        curInput=nInput(inInputSet{ii,1},obj,inInputSet{ii,2});
                        eval(strcat('inputHandle.',char(curInput),'=curInput;'));
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {symbol,description,val;...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {symbol,description,val;...}'));
            end
        end
        
        function varargout=genState(obj,inStateSet)
            varargout{1}=[];
            if isa(inStateSet,'cell')
                signalSize=size(inStateSet);
                if signalSize(2)==3
                    varargout=cell(signalSize(1),1);
                    for ii=1:signalSize(1)
                        curState=nState(inStateSet{ii,1},obj,inStateSet{ii,2},inStateSet{ii,3});
                        varargout{ii}=curState;
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {(name1) (des1) (order1);(name2) (des2) (order2);...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {(name1) (des1) (order1);(name2) (des2) (order2);...}'));
            end
        end
        
        function varargout=genMMSSolver(obj,inMMSSet)
            varargout{1}=[];
            if isa(inMMSSet,'cell')
                signalSize=size(inMMSSet);
                if signalSize(2)==3
                    varargout=cell(signalSize(1),1);
                    for ii=1:signalSize(1)
                        curState=nMMS(inMMSSet{ii,1},obj,inMMSSet{ii,2},inMMSSet{ii,2});
                        varargout{ii}=curState;
                    end
                else
                    error(obj.msgStr('Error','Input Data in the form of {(name1) (MMS order 1)  (MMS Term 1); (name2) (MMS order 2)  (MMS Term 2);...}'));
                end
            else
                error(obj.msgStr('Error','Input Data in the form of {(name1) (MMS order 1)  (MMS Term 1); (name2) (MMS order 2)  (MMS Term 2);...}'));
            end
        end
        
        function output=getParamVector(obj)
            output=obj.Param.get(obj.Param.Content);
            if(isempty(output))
                return;
            end
            output=[output{:}].';
        end
        
        function output=getInputVector(obj)
            output=obj.Input.get(obj.Input.Content);
            if(isempty(output))
                return;
            end
            output=[output{:}].';
        end
        
        function [Sym,Val,Order]=getStateVector(obj)
            Sym=[];
            Val=[];
            Order=[];
            signalList=obj.State.get(obj.State.Content);
            if(isempty(signalList))
                return;
            end
            
            for ii=1:numel(signalList)
                for jj=0:(signalList{ii}.Order-2)
                    Sym=[Sym;signalList{ii}.dot(jj)];
                    Val=[Val;signalList{ii}.dot(jj+1)];
                    Order=[Order;signalList{ii}.ExprOrder-signalList{ii}.Order+jj+1];
                end
                Sym=[Sym;signalList{ii}.dot(signalList{ii}.Order-1)];
                Val=[Val;signalList{ii}.Expression];
                Order=[Order;signalList{ii}.ExprOrder];
            end
        end
        
        function obj=init(obj,extraFixPointCondition)
            [StateVec,ExprVec]=obj.getStateVector();
            InputVec=obj.getInputVector();
            if isempty(InputVec)
                InputVec=[];
            end
            
            if isempty(extraFixPointCondition)||isa(extraFixPointCondition,'sym')
                FixedPointStruct=solve([subs(ExprVec==0,InputVec,0*InputVec);extraFixPointCondition],StateVec,'real',true,'ReturnConditions',true);
                rootSize=numel(eval("FixedPointStruct."+char(StateVec(1))));

                obj.FixedPoint=sym(0)*zeros(numel(StateVec),rootSize);
                obj.FixedPointCondition=sym(0)*zeros(1,rootSize);

                for kk=1:numel(StateVec)
                    obj.FixedPoint(kk,:)=eval("FixedPointStruct."+char(StateVec(kk))).';
                end
                obj.FixedPointCondition=FixedPointStruct.conditions.';
                obj.FixedPoint=subs(obj.FixedPoint,FixedPointStruct.parameters,FixedPointStruct.parameters*NaN);
                jj=0;
                for ii=1:rootSize
                    if ~any(isnan(obj.FixedPoint(:,ii)))
                        jj=jj+1;
                        obj.FixedPoint(:,jj)=obj.FixedPoint(:,ii);
                        obj.FixedPointCondition(jj)=obj.FixedPointCondition(ii);
                    end
                end
                obj.FixedPoint=jSimplify(obj.FixedPoint(:,1:jj));
                obj.FixedPointCondition=simplify(obj.FixedPointCondition(1:jj));
            end
            obj.setInit(true);
        end
        
        function obj=setOrder(obj,inOrder)
            if floor(inOrder)>obj.Order
                obj.Order=floor(inOrder);
            end
        end
        
        function obj=setInit(obj,inBool)
            obj.IsInit=inBool;
        end
    end
end
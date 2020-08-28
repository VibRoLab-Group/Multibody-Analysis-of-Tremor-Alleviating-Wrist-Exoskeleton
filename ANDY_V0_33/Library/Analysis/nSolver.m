classdef nSolver < jObj
% Base object of solvers
    properties
        System;
        
        FixedPoint;
        FixedPointCondition;
        
        SolverScaleVar;  
        
        NDimTimeVar;
        NDimFreq;
        NDimOrder; %reference order of nondimensionlization, the scaling is 1 for order NDimOrder
        NDimScale;
              
        StateNDim;
        ExpressionNDim;
        StateJacobian;
        InputJacobian;
        HighOrderTerm;
        
        IsInit;
        %This also indicate that the definition of states need to be
        %properly matched in orders.
    end
    
    
    methods(Access=public)
        function obj=nSolver(inName,inSys)
            obj@jObj(inName);
            if ~isempty(strfind(inName,'_'))
                error(obj.msgStr('Error','''_''is forbidden in Solver Naming!'));
            end
            
            if(isa(inSys,'nSystem'))
                obj.System=inSys;
                obj.NDimTimeVar=sym(strcat(char(inSys.TimeVar),inName));
                obj.SolverScaleVar=sym(strcat(char(inSys.ScaleVar),inName));
            else
                error(obj.msgStr('Error','InSys is not nSystem Object!'));
            end
            
            
            obj=obj.setNDimParam(sym(1),1);
            
            [state]=obj.System.getStateVector();
            stateND=state;
            for ii=1:numel(state)
                stateND(ii)=sym(strrep(char(state(ii)),'__dt_',"__d"+char(obj.NDimTimeVar)+"_"));
            end
            obj.StateNDim=stateND;
            
            obj.FixedPoint=obj.StateNDim*0;
            obj.FixedPointCondition=sym(1);
            
            obj.ExpressionNDim=[];
            obj.StateJacobian=[];
            obj.InputJacobian=[];
            obj.HighOrderTerm=[];
            
            obj.IsInit=false;
            
        end
        
        function obj=setFixedPoint(obj,inNum,inExtraFix)
            if obj.System.IsInit==false
                error(obj.msgStr('Error','System not initialized, cannot set fixed point!'));
            end
            
            [state]=obj.System.getStateVector();
            stateNum=numel(state);
            
            if inNum==0
                if isa(inExtraFix,'sym') && numel(inExtraFix) == stateNum 
                    obj.FixedPoint=inExtraFix;
                else
                    error(obj.msgStr('Error','Manually set fixed point does not fit in size or is not a Sym type!'));
                end
            elseif inNum>0&&inNum<=numel(obj.System.FixedPointCondition)
                obj.FixedPoint=obj.System.FixedPoint(:,inNum);
                obj.FixedPointCondition=obj.System.FixedPointCondition(inNum);
            else
                error(obj.msgStr('Error',"System only has "+num2str));
            end
            
        end
        
        function obj=setNDimParam(obj,inFreq,inOrder)
            sysOrder=obj.System.Order;
            [~,~,order]=obj.System.getStateVector();
            
            if sysOrder<inOrder||inOrder<=0
                error(obj.msgStr('Error','Input nondimensionalization frequency should be an positive integer no larger than system time order!'));
            else
                if isa(inFreq,'sym')
                    obj.NDimFreq=inFreq;
                    obj.NDimOrder=inOrder;
                else
                    error(obj.msgStr('Error','The nondimensionalization frequency is not a Sym type!'));
                end
            end
            
            obj.NDimScale=inFreq.^(order-inOrder-1);
            
        end
        
            
        function obj=setInit(obj,inBool)
            obj.IsInit=inBool;
        end
        
        function obj=init(obj)
            [state,expr]=obj.System.getStateVector();
            
            stateND=obj.StateNDim;
            epi=obj.SolverScaleVar;
            scaleMat=diag(obj.NDimScale);
            exprND=(scaleMat*obj.NDimFreq*epi)\(subs(expr,state,scaleMat*(epi*stateND+obj.FixedPoint)));
            
            obj.StateNDim=stateND;
            obj.ExpressionNDim=exprND;
            obj.StateJacobian=jacobian(subs(exprND,epi,0),stateND);
            input=obj.System.getInputVector();
            if isempty(input)
                obj.InputJacobian=[];
                obj.HighOrderTerm=exprND-obj.StateJacobian*stateND;
            else
                obj.InputJacobian=subs(jacobian(subs(exprND,epi,0),input),input,0*input);
                obj.HighOrderTerm=exprND-obj.StateJacobian*stateND-obj.InputJacobian*input;
            end
            
            obj.setInit(true);
        end
    end
end
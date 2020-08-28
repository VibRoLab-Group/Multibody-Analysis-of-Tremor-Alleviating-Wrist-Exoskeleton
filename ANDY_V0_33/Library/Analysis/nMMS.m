classdef nMMS < nSolver
    
    properties(SetAccess=protected)
        
        MMSOrder;
        MMSTerm;
        MMSState;
        MMSTime;
        
        ExpressionMMS;
        ExpressionMMSdtLeft;
        ExpressionMMSdtRight;        
    end
    
    methods(Access=public)
        
        function obj=nMMS(inName,inSys,inOrder,inTerm)
            obj@nSolver(inName,inSys);
            inSys.MMSSolver.reg(obj);
            
            [state]=obj.System.getStateVector();
            obj.MMSOrder=inOrder;
            obj.MMSTerm=inTerm;
            obj.MMSTime=sym(0)*zeros(1,inOrder+1);
            obj.MMSState=sym(0)*zeros(numel(obj.StateNDim),inTerm+inOrder);
            for ii=0:(inTerm+inOrder-1)
                obj.MMSTime(ii+1)=sym(strcat(char(obj.NDimTimeVar),num2str(ii)));
                for jj=1:numel(state)
                    obj.MMSState(jj,ii+1)=sym(strrep(char(state(jj)),"__dt_0_","__"+char(inName)+"__SC"+num2str(ii)+"_"));
                end
            end
            
            for ii=0:(inTerm+inOrder-1)
                for jj=1:numel(state)
                    for kk=0:inOrder
                        obj.MMSState(jj,ii+1)=sym(char(obj.MMSState(jj,ii+1))+"_d"+char(obj.MMSTime(kk+1))+"_0_");
                    end
                end 
            end
            
            obj.ExpressionMMS=obj.MMSState*(obj.SolverScaleVar.^[0:(inTerm+inOrder-1)].');
            obj.ExpressionMMSdtLeft=sym(zeros(numel(state),inTerm+inOrder));
            exprDt=obj.pDiffMMS(obj.ExpressionMMS,1);
            for ii=1:(numel(state))
                exprCoeffs=coeffs(exprDt(ii),obj.SolverScaleVar);
                obj.ExpressionMMSdtLeft(ii,:)=exprCoeffs(1:inTerm+inOrder);
            end
        end
        
        function obj=MMSInit(obj,inNum,inExtraFix,inFreq,inOrder)
            obj.setFixedPoint(obj,inNum,inExtraFix);
            if ~isempty(inFreq)||~isempty(inOrder)
                obj.setNDimParam(obj,inFreq,inOrder);
            end
            obj.init();
            
        end
        
        function output=pDiffMMS(obj,inEq,inOrder)
            output=inEq;
            for jj=1:inOrder
                output0=output;
                output=0;
                for ii=1:numel(obj.MMSTime)
                    output=output+pDiff(output0,obj.MMSTime(ii))*(obj.SolverScaleVar)^(ii-1);
                end
            end
        end
        
    end
end
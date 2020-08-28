classdef nInput < nVar
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=nInput(inName,inSys,inDes)
            obj@nVar(inName,inSys,inDes);
            inSys.Input.reg(inDes,obj);
        end
    end
end
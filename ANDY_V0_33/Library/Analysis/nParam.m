classdef nParam < nVar
    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=nParam(inName,inSys,inDes)
            obj@nVar(inName,inSys,inDes);
            inSys.Param.reg(inDes,obj);
        end
    end
end
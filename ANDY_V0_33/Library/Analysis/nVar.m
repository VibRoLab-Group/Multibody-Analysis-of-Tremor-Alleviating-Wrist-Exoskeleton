classdef nVar < jObj & sym
    properties(SetAccess=protected)
        System;
    end
    
    methods(Access=public)
        function obj=nVar(inName,inSys,inDes)
            obj@jObj(inDes);
            obj@sym(inName,'real');
            
            if ~isempty(strfind(inName,'_'))
                error(obj.msgStr('Error','''_''is forbidden in Symbolic Variable Naming!'));
            end
            
            if(isa(inSys,'nSystem'))
                obj.System=inSys;
                obj.System.setInit(false);
            else
                error(obj.msgStr('Error','InSys is not jSys Object!'));
            end
        end
    end
end
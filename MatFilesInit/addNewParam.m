function structureChanged = addNewParam(structureToChange,field,defaultValue,paramDescription,paramType,fileCfg,varargin)

sourceForValue = 0; % 0: default
value = defaultValue;
valueInCfg = searchParamInCfgFile(fileCfg,field,paramType);
if valueInCfg ~= -1
    value = valueInCfg;
    sourceForValue = 1; % 1: file config
end
for i=1:(length(varargin{1})-1)/2
    if strcmpi(char(varargin{1,1}(2*i)),field)
        value = cell2mat(varargin{1,1}(2*i+1));
        sourceForValue = 2; % 2: command line
    end
end
structureChanged = setfield(structureToChange,field,value);

% Print to command window
fprintf('%s:\t',paramDescription);
fprintf('[%s] = ',field);
if strcmpi(paramType,'integer')
    if ~isnumeric(value) || mod(value,1)~=0
        error('Error: parameter %s must be an integer.',field);
    end
    fprintf('%.0f ',value);
elseif strcmpi(paramType,'double')
    if ~isnumeric(value)
        error('Error: parameter %s must be a number.',field);
    end
    fprintf('%f ',value);
elseif strcmpi(paramType,'string')
    if ~ischar(value)
        error('Error: parameter %s must be a string.',field);
    end
    fprintf('%s ',value);
elseif strcmpi(paramType,'bool')
    
    if ~islogical(value)
        error('Error: parameter %s must be a boolean.',field);
    end
    if value == true
        fprintf('true ');
    else
        fprintf('false ');
    end
else
    error('Error in addNewParam: paramType can be only integer, double, string, or bool.');
end
if sourceForValue==0
    fprintf('(default)\n');
elseif sourceForValue==1
    fprintf('(file %s)\n',fileCfg);
else
    fprintf('(command line)\n');
end
    


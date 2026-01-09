function S = extract_time_simulation_data(filename)
% Extract simulation data from file into structure S
% USAGE:
%   S = extract_simulation_data_gen(filename)


% initialise results structure
S = struct;

% read metadata
metafile = [filename(1:end-3) 'meta'];
fileID = fopen(metafile,'r');
C = textscan(fileID,'%s %f','HeaderLines',6);
fclose(fileID);

% read simulation output data into matrix A
fileID = fopen(filename,'r');
temp = fgetl(fileID);
ncols = numel(regexp(temp,';\t','split'));
formatSpec = [repmat('%f; ',1,ncols-1) ' %f \n'];
fseek(fileID,0,'bof');
A = fscanf(fileID,formatSpec,[ncols inf]);
A = A';
fclose(fileID);

% extract time
time = A(:,1);

% identify task start points
I = find(time==0); % indices of first time step of each task
ntasks = length(I);  % number of tasks

% number of data fields
nfields = length(C{1})-2;

% cycle through data fields
colcounter = 1;
for k = 1:nfields

    % extract name of field
    fldname = C{1}{k};
    if strcmp(fldname(1:3),'d->')
        fldname = fldname(4:end-1);
    elseif strcmp(fldname(1:2),'c.')
        fldname = fldname(3:end-1);
    else
        fldname = fldname(1:end-1);
    end
    if strcmp(fldname,'targ_xpos')      % correct name for back compatibility
        fldname = 'xtarg';
    elseif strcmp(fldname, 'grip_xpos') % correct name for back compatibility
        fldname = 'xpos';
    end
    
    % number of columns corresponding to field
    nfieldcol = C{2}(k);

    % get row indices corresponding to task and extract data
    for i = 1:ntasks
        if i<ntasks
            inds = I(i):(I(i+1)-1);
        else
            inds = I(i):length(time);
        end
        S(i).(fldname) = A(inds,colcounter:colcounter+nfieldcol-1);
    end
    % keep track of columns of data read
    colcounter = colcounter + nfieldcol;
end

% extract initial position and clean up target position data
for i = 1:ntasks
    S(i).xinit = S(i).xpos(1,:);
    S(i).xtarg(2:end,:) = [];
end

% clean up
clear A;


end


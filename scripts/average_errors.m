function [est,emv] = average_errors(S)
% AVERAGE_ERRORS computes homing in error (ehi) and tracking error (emv) values from simulation data in input
% structure S.
% USAGE:
%   [est,emv] = average_errors(S)

% extract fieldname hierarchy until data level is found
FN = {};
FN{1} = fieldnames(S);
i = 1;
while ~sum(ismember(FN{i},'qpos')) && i<2
    i = i+1;
    FN{i} = fieldnames(S.(FN{i-1}{1}));
end

if ~sum(ismember(FN{i},'qpos'))
    error('could not find data level with field qpos')
else
    data_i = i;
end

if data_i>1 % data not on first level
    m = nan(data_i-1,1);
    for j = 1:data_i-1
        m(j) = length(FN{j});
    end
    n = length(S.(FN{1}{1}));  % fix this to work with data_i>2
else
    m = 1;
    n = length(S);
end


%%

est = nan(n,m);
emv = nan(n,m);

for j=1:m
    if data_i>1
        Stemp = S.(FN{1}{j});
    else
        Stemp = S;
    end
    for i=1:n
        xerror0 = Stemp(i).xtarg-Stemp(i).xinit;
        dt0 = norm(xerror0);
        Td = 1.712*dt0^(1/3);
        
        if isfield(Stemp,'xerror_norm')
            est(i,j) = mean(Stemp(i).xerror_norm(Stemp(i).time>Td & Stemp(i).time>Td));
        else
            xerror_norm = sqrt(sum((Stemp(i).xpos-Stemp(i).xtarg).^2,2));
            est(i,j) = mean(xerror_norm(Stemp(i).time>Td));
        end
        
        xerror = Stemp(i).xtarg-Stemp(i).xpos;
        tau = [0; Stemp(i).time(1:end-1)]/Td;  % mismatch between time at MuJoCo computation
        wn = (6*tau.^5-15*tau.^4+10*tau.^3);
        wn(tau>=1) =1;
        error_d_norm = sqrt(sum((xerror + (wn-1).*xerror0).^2,2));
        emv(i,j) = mean(error_d_norm);  
    end
end

end


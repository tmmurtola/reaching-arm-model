function [inWS] = is_in_workspace(X,varargin)
% IS_IN_WORKSPACE Checks if points in X are within arm model's workspace
%    X is n-by-2 with x-coordinates in first and y-coordinates in second
%    column. Optional argument: 1D or 2D vector of buffer zone used around
%    geometric boundary. 1D is used for all boundaries, 2D is of the form
%    [outer_buffer inner_buffer] for outer and inner (+chest link) arcs, respectively. 

inWS = true(size(X,1),1);

if nargin>1
    buffer = varargin{1};
    if length(buffer)>1
        buff_out = buffer(1);
        buff_in = buffer(2);
    else
        buff_out = buffer(1);
        buff_in = buff_out;
    end
else
    buff_out = 0;
    buff_in = 0;
end

link_lengths = [0.44 0.32 0.25 0.20]';


% outer arc 0 (extended arm, shoulder flexion)
shoulder_pos = [0 0];
Ltot = sum(link_lengths(2:end));

Xtemp = X-shoulder_pos;
[theta0, r0] = cart2pol(Xtemp(:,1),Xtemp(:,2));

inWS(r0>(Ltot+buff_out)) = 0;

% inner arc 2 (flexed arm, shoulder flexion)
[ax, ay] = pol2cart(deg2rad(0),link_lengths(2));
[bx, by] = pol2cart(deg2rad(0+120),link_lengths(3));
[cx, cy] = pol2cart(deg2rad(0+120+70),link_lengths(4));
grip_pos = shoulder_pos + [ax ay] + [bx by] + [cx cy];
[~, rg] = cart2pol(grip_pos(1), grip_pos(2));

inWS(r0<(rg+buff_in)) = 0;


% outer arc 1 (flexed shoulder, elbow flexion)
 [elbow_x, elbow_y] = pol2cart(deg2rad(120),link_lengths(2));
 elbow_pos = [elbow_x elbow_y];
 Leg = sum(link_lengths(3:end));
 
 ind = and(theta0>deg2rad(120),theta0<=pi);
 Xtemp = X-elbow_pos;
 [~, r1] = cart2pol(Xtemp(:,1),Xtemp(:,2));
 
 inWS(and(ind,r1>(Leg+buff_out))) = 0;


 % inner arc 3 (extended shoulder, elbow flexion)
 [elbow_x, elbow_y] = pol2cart(deg2rad(-20),link_lengths(2));
 elbow_pos = [elbow_x elbow_y];
 grip_pos = [bx by] + [cx cy];
 [~, rg] = cart2pol(grip_pos(1), grip_pos(2));
 
 Xtemp = X-elbow_pos;
[~, r3] = cart2pol(Xtemp(:,1),Xtemp(:,2));

inWS(r3<(rg+buff_in)) = 0;

% upper chest link
ind = and(and(X(:,1)<=0, X(:,1)>-0.445),X(:,2)<(0.045+buff_in));
inWS(ind) = 0;


end


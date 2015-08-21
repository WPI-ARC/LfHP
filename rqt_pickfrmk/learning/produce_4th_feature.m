fid = fopen('distances_exp_3.txt');
distances = [];
sig_dist = [];
sum_dist = [];
tline = fgetl(fid);
i = 1;
%min = -1000
while ischar(tline)
    distances = [distances textscan(tline,'%f','Delimiter',',');]
    %t_min = min(distances)
    %if t_min < min
    %    min = t_min
    %end
    distances{i} = distances{i} %+ 0.08
    tline = fgetl(fid);
    sig_dist = (-1./(1+exp(-20*distances{i}))+1)
    sum_dist = [sum_dist; sum(sig_dist)];
    i = i + 1;
end
fclose(fid);

% distances=csvread('4th_feature_non_fixed_non_neg.txt')';
% sig_dist = (-1./(1+exp(-40*distances{1}))+1);
% a = sum(sig_distances, 1);
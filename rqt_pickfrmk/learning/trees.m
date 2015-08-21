% upload raw data
features=csvread('time_pathlength_orient_exp_3.txt')';
%distances=csvread('4th_feature_non_fixed_non_neg.txt')';
%summ_dist = sum(distances);
%features = [features summ_dist'];

fid = fopen('distances_exp_3.txt');
distances = [];
sig_dist = [];
sum_dist = [];
tline = fgetl(fid);
i = 1;
while ischar(tline)
    distances = [distances textscan(tline,'%f','Delimiter',',');]
    tline = fgetl(fid);
    sig_dist = (-1./(1+exp(-90*distances{i}))+1)
    sum_dist = [sum_dist; sum(sig_dist)];
    i = i + 1;
end
fclose(fid);
features = [features sum_dist];

% rank raw data
Ranks=csvread('ranking_exp3_refined.txt');
Ranks = linspace(1,32,32)'; %ranks for test data (meaningless)
%load('Ranks_100');
X = [];
Y = linspace(1,0,32);
for i = 1:32
   ind = Ranks(i,1);
   X = [X features(ind,:)'];
end
cat = false;

if cat
    Y = [1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 4 4 4 4 4 4 4 4 4 4];    
end
X = X';
Y = Y';


% sigmoid features
x3 = (-1./(1+exp(-2*X(:,3)))+1)*100000;
% x4 = (-1./(1+exp(-1.3*X(:,4)))+1)*20;
x4 =  (-1./(1+exp(-X(:,4)/3.5))+1);

% data_sig = [X(:,1) X(:,2) x3 x4 Y [1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 4 4 4 4 4 4 4 4 4 4]'];
data = [X(:,1) X(:,2) X(:,3) X(:,4) Y ];% [1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 4 4 4 4 4 4 4 4 4 4]'];

if false
    % calculation for the data that came from path length
    a = linspace(1,32,32)';
    temp = [X(:,2)  a];
    t = sortrows(temp);
    Y_ = t(:,2);
    sum_ = 0;
    for i = 1:size(Y_)
        %if Y1(i) > max(Y2)
        %    sum = sum + abs(max(Y2) - Y1(i));
        %    never_found = false;
        %end
        %if Y1(i) < min(Y2)
        %    sum = sum + abs(min(Y2) - Y1(i));
        %    never_found = false;
        %end
        pos = find(a == Y_(i));
        if pos ~= i
            sum_ = sum_ + abs(pos - i);
        %    never_found = false;
        end

    end
end


% M = ranked;
% b = zeros(1,32);
% b1 = zeros(1,32);
% tree1 = RegressionTree.fit(X, Y, 'Minparent', 2, 'Prune', 'on');
% %tree1 = ClassificationTree.fit(X, Y, 'Minparent', 2, 'Prune', 'on');
% tree2 = tree1.prune('level',0);
% view(tree2,'mode','graph');
% results_tree = [];

%b = predict(tree2, X);

%b1 = mvregress();


%X = X(:,1:3);
for i = 1:32
    train_X = [X(1:(i-1), :); X((i+1):30, :) ];
    train_Y = [Y(1:(i-1), :); Y((i+1):30, :) ];
    if cat
        tree1 = ClassificationTree.fit(train_X, train_Y, 'Minparent', 2, 'Prune', 'on');
    else
        tree1 = RegressionTree.fit(train_X, train_Y, 'Minparent', 3, 'Prune', 'on');
    end
    
    tree2 = tree1.prune('level',1);
    view(tree2,'mode','graph');
    results_tree = [];

    b(i) = predict(tree2, X(i,:));
    coef = mvregress(X,Y);
    b1(i) = X(i,:)*coef;
end
b

% reproduce the ranking from continious output
[sorted, ind] = sort(b, 'descend');
[sorted_, ind2] = sort(b1, 'descend');
% not working code further
% [sorted2, Y_] = sort(res2);
% Y2 = data.result;
Y_ = ind;%fliplr(ind);
Y_ = Y_';
sum = 0;
%never_found = true;
for i = 1:size(Y_)
    %if Y1(i) > max(Y2)
    %    sum = sum + abs(max(Y2) - Y1(i));
    %    never_found = false;
    %end
    %if Y1(i) < min(Y2)
    %    sum = sum + abs(min(Y2) - Y1(i));
    %    never_found = false;
    %end
    pos = find(a == Y_(i));
    if pos ~= i
        sum = sum + abs(pos - i);
    %    never_found = false;
    end

end

a =5;
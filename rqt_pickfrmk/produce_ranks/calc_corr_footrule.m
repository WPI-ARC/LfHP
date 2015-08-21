%features=csvread('cros_val_exp1_test_trees.csv')';
features=csvread('ranking_32_exp3.txt')';
[s, ind1] = sort(features(3,:),'descend');
%[s, ind1] = sort(features(2,:));
f = features(:,ind1);

d = corr(features(2,:)', features(3,:)');

[sorted, ind2] = sort(f(3,:));

Y = linspace(1,32,32);
sum = 0;
for i = 1:size(Y,2)
    %if Y1(i) > max(Y2)
    %    sum = sum + abs(max(Y2) - Y1(i));
    %    never_found = false;
    %end
    %if Y1(i) < min(Y2)
    %    sum = sum + abs(min(Y2) - Y1(i));
    %    never_found = false;
    %end
    pos = find(Y(i) == ind2(i));
    if pos ~= i
        sum = sum + abs(pos - i);
    %    never_found = false;
    end

end
sum
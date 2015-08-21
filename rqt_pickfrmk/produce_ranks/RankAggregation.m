function RankAggregation()

    data = csvread('results_150.txt')'
    
    rank_size = 30;
    for p = 1:10
        Scores = zeros(rank_size,2);
        %name = strcat('mat_files/Preferences_',int2str(p));
        %load(name);
        Pref = data;
        n = size(Pref);
        for i = 1:rank_size
            temp_score = 0;
            for j = 1:n(2)
                c_ij = 0;
                if Pref(1,j) == i
                    c_ij = 1;
                    temp_score = temp_score + (2*c_ij -1);
                elseif Pref(2,j) == i
                    c_ij = 0;
                    temp_score = temp_score + (2*c_ij -1);
                end
            end
            Scores(i, :) = [i temp_score];
        end
        Ranks = sortrows(Scores,-2);
        name = strcat('Ranks_',int2str(p));
        save(name, 'Ranks');
    end
end
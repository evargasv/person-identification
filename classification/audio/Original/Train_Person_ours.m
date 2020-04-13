function Person = Train_Person_ours(k)

% Our dataset
if    k >= 1    &&   k <= 9
    Person = 1;
elseif k >= 10   &&  k <= 21
    Person = 2;
elseif k >= 22  &&   k <= 30
    Person = 3;
elseif k >= 31  &&   k <= 39
    Person = 4;
elseif k >= 40  &&   k <= 48
    Person = 5;
elseif k >= 49  &&   k <= 57
    Person = 6;
elseif k >= 58  &&   k <= 69
    Person = 7;
elseif k >= 70  &&   k <= 78
    Person = 8;
elseif k >= 79  &&   k <= 87
    Person = 9;
elseif k >= 88  &&   k <= 96
    Person = 10;
elseif k >= 97  &&   k <= 108
    Person = 11;
elseif k >= 109  &&   k <= 117
    Person = 12;
elseif k >= 118  &&   k <= 123
    Person = 13;
elseif k >= 124  &&   k <= 135
    Person = 14;
elseif k >= 136  &&   k <= 144
    Person = 15;
end

% % English ours
% if    k == 1    || k==2 || k==3 || k==10 || k==11 || k==12
%     Person = 1;
% elseif k ==4 || k==5 || k==6 || k==13 || k==14 || k==15
%     Person = 2;
% elseif k ==7 || k==8 || k==9 || k==16 || k==17 || k==18
%     Person = 3;
% elseif k ==19 || k==20 || k==21 || k==22 || k==23 || k==24
%     Person = 4;
% end


% % Ours - Very new 
% if    k >= 1    &&   k <= 3
%     Person = 1;
% elseif k >= 4   &&  k <= 6
%     Person = 2;
% elseif k >= 7  &&   k <= 9
%     Person = 3;
% elseif k >= 10  &&   k <= 12
%     Person = 4;
% end



end
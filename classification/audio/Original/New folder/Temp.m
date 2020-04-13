msg = sprintf('Speaker %d matches with speaker %d', k, k1);
    if(1<=k && k<=70 &&   1<=k1 && k1<=70)
        accuracy(1) = accuracy(1)+1;
    elseif(71<=k && k<=140 &&   71<=k1 && k1<=140)
        accuracy(2) = accuracy(2)+1;
    elseif(143<=k && k<=215 && 143<=k1 && k1<=212)
        accuracy(3) = accuracy(3)+1;
     elseif(216<=k && k<=282 && 213<=k1 && k1<=280)
         accuracy(4) = accuracy(4)+1;
     elseif(283<=k && k<=352 && 281<=k1 && k1<=351)
        accuracy(5) = accuracy(5)+1;
    elseif(353<=k && k<=423 && 352<=k1 && k1<=421)
         accuracy(6) = accuracy(6)+1;
    elseif (424<=k && k<=505 && 422<=k1 && k1<=491)
        accuracy(7) = accuracy(7)+1;
    elseif(506<=k && k<=564 && 492<=k1 && k1<=562)
        accuracy(8) = accuracy(8)+1;
    elseif(565<=k && k<=640 && 563<=k1 && k1<=632)
        accuracy(9) = accuracy(9)+1;
    elseif(641<=k && k<=705 && 633<=k1 && k1<=702)
        accuracy(10) = accuracy(10)+1;
    elseif(705<=k && k<=776 && 703<=k1 && k1<=773)
        accuracy(11) = accuracy(11)+1;
    end
    disp(msg);
    
       %     speaker 1 : 1-> 70
    %     speaker 2: 71->142
    %     speaker 3: 143->214
    %     speaker 4: 215->285
    %     speaker 5: 286->355
    %     speaker 6: 356->427
    %     speaker 7: 428->498
    %     speaker 8: 499->570
    %     speaker 9: 571->641
    %     speaker 10:642->713
    %     speaker 11: 714->785
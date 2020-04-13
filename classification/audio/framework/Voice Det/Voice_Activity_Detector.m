function New_Audio = Voice_Activity_Detector (Audio_Data, fs)

[vs,zo]=vadsohn(Audio_Data, fs, 'a');
[r, c] = find(vs(:, 1)==1);

New_Audio = zeros(size(r));
for i = 1:size(r, 1)
    New_Audio(i) = Audio_Data(r(i));
end

end
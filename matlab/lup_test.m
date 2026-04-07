A = single([4, 3, 2, 2, 1, 5; %[output:group:740aab5e] %[output:68a7719d]
     3, 1, 1, 1, 0, 2; %[output:68a7719d]
     2, 1, 3, 2, 1, 3; %[output:68a7719d]
     2, 1, 2, 3, 1, 4; %[output:68a7719d]
     1, 0, 1, 1, 2, 2; %[output:68a7719d]
     5, 2, 3, 4, 2, 1]) %[output:group:740aab5e] %[output:68a7719d]

inv_A = inv(A) %[output:0595da12]
inv_a_re = reshape(inv_A, [1 36]) %[output:96327fa5]
det_A = det(A) %[output:9caca85d]

b = [2; 1; 3; 7; 6; 9] %[output:8376aad2]
A = [10, 2, 1, 1; %[output:group:22c3f20f] %[output:10103f12]
     3, 9, 2, 1; %[output:10103f12]
     2, 1, 8, 2; %[output:10103f12]
     1, 1, 1, 7] %[output:group:22c3f20f] %[output:10103f12]
b = [2 1 3 7] %[output:0d5e87ea]


x = b / A %[output:50d6c4e1]
 
x * A  %[output:5c08bde6]

lu(A) %[output:8fff5b2a]
%%
A = [10, 2, 1, 1; %[output:group:3cb79c6e] %[output:68a69c79]
      3, 1, 2, 1; %[output:68a69c79]
      2, 7, 1, 2; %[output:68a69c79]
      1, 1, 5, 7] %[output:group:3cb79c6e] %[output:68a69c79]
b = [2 1 3 7] %[output:00f0e33f]


x = b / A %[output:53d74720]
 
x * A  %[output:19e893e7]

lu(A) %[output:4859b292]
%%
A = [1, 2, 3; %[output:group:8de7bb43] %[output:57c64551]
     8, 1, 7; %[output:57c64551]
     3, 7, 2; %[output:57c64551]
     9, 5, 8]  %[output:group:8de7bb43] %[output:57c64551]

B = [5, 0, 3; %[output:group:6ef06147] %[output:052e8973]
     0, 2, 1; %[output:052e8973]
     3, 1, 7] %[output:group:6ef06147] %[output:052e8973]

abat = A * B * A.' %[output:09115a50]
abat_re = reshape(abat, [1 16]) %[output:69a186fd]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline"}
%---
%[output:68a7719d]
%   data: {"dataType":"matrix","outputData":{"columns":6,"header":"6×6 single matrix","name":"A","rows":6,"type":"single","value":[["4","3","2","2","1","5"],["3","1","1","1","0","2"],["2","1","3","2","1","3"],["2","1","2","3","1","4"],["1","0","1","1","2","2"],["5","2","3","4","2","1"]]}}
%---
%[output:0595da12]
%   data: {"dataType":"matrix","outputData":{"columns":6,"header":"6×6 single matrix","name":"inv_A","rows":6,"type":"single","value":[["-0.1337","0.5581","-0.0930","-0.1221","0.1453","0.0291"],["0.5581","-0.7209","-0.0465","-0.1860","-0.3023","0.1395"],["-0.0930","-0.0465","0.6744","-0.3023","-0.1163","-0.0233"],["-0.1221","-0.1860","-0.3023","0.5407","-0.2151","0.1570"],["0.1453","-0.3023","-0.1163","-0.2151","0.4942","0.0988"],["0.0291","0.1395","-0.0233","0.1570","0.0988","-0.1802"]]}}
%---
%[output:96327fa5]
%   data: {"dataType":"matrix","outputData":{"columns":36,"header":"1×36 single row vector","name":"inv_a_re","rows":1,"type":"single","value":[["-0.1337","0.5581","-0.0930","-0.1221","0.1453","0.0291","0.5581","-0.7209","-0.0465","-0.1860","-0.3023","0.1395","-0.0930","-0.0465","0.6744","-0.3023","-0.1163","-0.0233","-0.1221","-0.1860","-0.3023","0.5407","-0.2151","0.1570","0.1453","-0.3023","-0.1163","-0.2151","0.4942","0.0988"]]}}
%---
%[output:9caca85d]
%   data: {"dataType":"textualVariable","outputData":{"header":"single","name":"det_A","value":"172.0000"}}
%---
%[output:8376aad2]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"b","rows":6,"type":"double","value":[["2"],["1"],["3"],["7"],["6"],["9"]]}}
%---
%[output:10103f12]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"A","rows":4,"type":"double","value":[["10","2","1","1"],["3","9","2","1"],["2","1","8","2"],["1","1","1","7"]]}}
%---
%[output:0d5e87ea]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"b","rows":1,"type":"double","value":[["2","1","3","7"]]}}
%---
%[output:50d6c4e1]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"x","rows":1,"type":"double","value":[["0.0663","-0.0349","0.2603","0.9211"]]}}
%---
%[output:5c08bde6]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"ans","rows":1,"type":"double","value":[["2.0000","1.0000","3.0000","7.0000"]]}}
%---
%[output:8fff5b2a]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"ans","rows":4,"type":"double","value":[["10.0000","2.0000","1.0000","1.0000"],["0.3000","8.4000","1.7000","0.7000"],["0.2000","0.0714","7.6786","1.7500"],["0.1000","0.0952","0.0961","6.6651"]]}}
%---
%[output:68a69c79]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"A","rows":4,"type":"double","value":[["10","2","1","1"],["3","1","2","1"],["2","7","1","2"],["1","1","5","7"]]}}
%---
%[output:00f0e33f]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"b","rows":1,"type":"double","value":[["2","1","3","7"]]}}
%---
%[output:53d74720]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"x","rows":1,"type":"double","value":[["0.5749","-1.6654","0.0536","1.1405"]]}}
%---
%[output:19e893e7]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"ans","rows":1,"type":"double","value":[["2.0000","1.0000","3.0000","7.0000"]]}}
%---
%[output:4859b292]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"ans","rows":4,"type":"double","value":[["10.0000","2.0000","1.0000","1.0000"],["0.2000","6.6000","0.8000","1.8000"],["0.1000","0.1212","4.8030","6.6818"],["0.3000","0.0606","0.3438","-1.7066"]]}}
%---
%[output:57c64551]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"A","rows":4,"type":"double","value":[["1","2","3"],["8","1","7"],["3","7","2"],["9","5","8"]]}}
%---
%[output:052e8973]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"B","rows":3,"type":"double","value":[["5","0","3"],["0","2","1"],["3","1","7"]]}}
%---
%[output:09115a50]
%   data: {"dataType":"matrix","outputData":{"columns":4,"name":"abat","rows":4,"type":"double","value":[["106","301","143","369"],["301","1015","394","1186"],["143","394","235","509"],["369","1186","509","1415"]]}}
%---
%[output:69a186fd]
%   data: {"dataType":"matrix","outputData":{"columns":16,"name":"abat_re","rows":1,"type":"double","value":[["106","301","143","369","301","1015","394","1186","143","394","235","509","369","1186","509","1415"]]}}
%---

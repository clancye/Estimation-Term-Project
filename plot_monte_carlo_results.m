clf
path = "/home/clancy/Projects/Estimation Project 2016/Performance Data/performance";
numberOfRuns = 5;
aggregateData = zeros(8,1);
for i = 0:numberOfRuns-1
  filename = strcat(path, mat2str(i),".txt");
  data = importdata(filename,"=");
  aggregateData = aggregateData + data.data;
  kbhit();
endfor
aggregateData = aggregateData/numberOfRuns;
disp(aggregateData);
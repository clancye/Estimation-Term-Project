clf
path = "/home/clancy/Projects/Estimation Project 2016/Performance Data/performance";
numberOfRuns = 5;
for i = 0:numberOfRuns-1
  filename = strcat(path, mat2str(i),".txt");
  disp(filename);
  data = importdata(filename,"=");
  plot(data.data(:,1));
  kbhit();
endfor
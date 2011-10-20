function t = testdaq()
try 
	daq1208fs('i');
catch
	lasterror.message
	return
end

data(1:256) = randperm(256) - 1;
data(257:512) = randperm(256) - 1;
t = zeros(512,1);
for i = 1:512
	tic
	daq1208fs('w', data(i));
	daq1208fs('w', 0);
	t(i) = toc
	sleep(.1)
end

daq1208fs('d');

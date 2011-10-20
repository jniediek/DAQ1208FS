function times = daq_latency_tests(dio)

trials = 50;
puts_per_cycle = 10;

times = zeros(trials*puts_per_cycle,1);

for j = 1:trials
    values = randi(255,[puts_per_cycle 1]);
    for i = 1:puts_per_cycle;
        tic;
        putvalue(dio,values(i));
        times((j-1)*trials + i) = toc;
        WaitSecs(3*rand);
    end
end
end
        
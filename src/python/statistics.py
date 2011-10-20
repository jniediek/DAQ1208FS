from time import time
import usb1208fs
usb1208fs.init()
li = []
for j in range(20):
	for i in range(256):
		a = time()
		usb1208fs.dset(i)
		c = time()
		li.append(c - a)
	sum = 0
	for i in range(len(li)):
		sum = sum + li[i]
	mean = sum/len(li)
	errsum = 0
	for i in range(len(li)):
		errsum = errsum + (li[i] - mean)**2
	meansqerror = errsum/len(li)
	print 'mean: ' + repr(mean) + '\nmeansqerror: ' + repr(meansqerror)
usb1208fs.deinit()

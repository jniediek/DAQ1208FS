#include "mex.h"

static struct {
	double * a;
	int len;
} wort = {NULL, 0};

void reallocWort(double c) {
	wort.a = (double *) mxRealloc(wort.a, (wort.len+1) * sizeof(double));
	mexMakeMemoryPersistent(wort.a);
	wort.a[wort.len] = c;
	wort.len++;
}

void myExit() {
	mxFree(wort.a);
}

void printit() {
	int i;
	for(i = 0; i < wort.len; i++) {
		mexPrintf("%g ", wort.a[i]);
	}
	mexPrintf("\n");
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	double x;
	if (nrhs != 1) {
		mexErrMsgTxt("1 Argument!\n");
	}
       	x = mxGetScalar(prhs[0]);
	reallocWort(x);
	mexAtExit(&myExit);
	printit();
	return;
}

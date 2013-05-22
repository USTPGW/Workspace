double invmat[100] = 
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
	 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
};

void inverse(double *matr, int n, double *inv)
{
	int i;
	int j;
	int k;
	
	// define temp matrix as input matrix for easy manipulation
	for(i=0;i<n;i++){
		for(j=0;j<n;j++){
			test[i*n+j] = matr[i*n+j];
		}
	}
	
	//rref
	for(i=0;i<(n-1);i++){
		for(k=1;k<(n-i);k++){
			d[0] = test[(i+k)*n+i]/test[i*n+i];
			for(j=i;j<n;j++){
						test[(i+k)*n + j] = test[(i+k)*n+j] - d[0]*test[i*n+j];
			}
			for(j=0;j<n;j++){
				invmat[(i+k)*n + j] = invmat[(i+k)*n+j] - d[0]*invmat[i*n+j];
			}
		}
	}
	
	//turn to identity
	for(i=n-1;i>-1;i--){
		for(j=0;j<n;j++){
			invmat[i*n+j] = invmat[i*n+j]/test[i*n+i];
		}
			test[i*n+i] = test[i*n+i]/test[i*n+i];
		for(j=i;j>0;j--){
			d[0] = test[(j-1)*n+i];
			test[(j-1)*n+i] = test[(j-1)*n+i] - test[(j-1)*n+i]*test[i*n+i];
			for(k=0;k<n;k++){
				invmat[(j-1)*n+k] = invmat[(j-1)*n+k] - d[0]*invmat[i*n+k];
			}
		}
	}
	
	for(i=0;i<n*n;i++){
		inv[i] = invmat[i];
	}
}
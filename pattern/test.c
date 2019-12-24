/*int n;
float qqq[3][5][8];
int fact()
{
	if (n == 1)
	{ 
		return n;
	}
	else

	{ 
		n =n-1;
		return (n*fact());
	}
}

int fib(int x){
	if(x==0||x==1)return 1;
	return fib(x-1)+fib(x-2);
}

int test(int a,int b,int c,int d,int e,int f,int g,int h,int i){
	if(i>10){
		i=i+100;
	}
	return i;
}

int ga(float tmp[3][5][8]){
	return tmp[1][2][3];
}
int gaa(float tmp[5][8]){
	return tmp[2][3];
}
int gaaa(float tmp[8]){
	return tmp[3];
}
float rrr(){
	return 1;
}
int MAIN(){
    int a0,b0,c0,d0,e0,f0,g0,h0,i0,j0,k0,l0,m0,o0,p0,q0,r0,s0,t0,u0,v0,w0,x0,y0,z0;
	int result;
	float ary[3][5][8];
	float ttt[8];
	ary[1][2][3]=9487;
	qqq[1][2][3]=555;
	ttt[3]=9292;
	write(rrr());
	write("\n");
	write(ary[1][2][3]);
	write("\n");
	write(ga(ary));
	write("\n");
	write(qqq[1][2][3]);
	write("\n");
	write(ga(qqq));
	write("\n");
	write(gaa(ary[1]));
	write("\n");
	write(gaa(qqq[1]));
	write("\n");
	write(gaaa(ary[1][2]));
	write("\n");
	write(gaaa(qqq[1][2]));
	write("\n");
	write(gaaa(ttt));
	write("\n");
	write(test(1,2,3,4,5,6,7,8,887));
	write("\n");
	for(a0=1;a0<10;a0=a0+1){
		write(fib(a0));
		write(" ");
	}
	write("\n");

	write("Enter a number:");

	n = read();
	n = n+1;
	if (n > 1)

	{ 
		result = fact();
	}
	else
	{ 
		result = 1;
	}
    z0=888;
	write("The factorial is ");
	write(result);
	write("\n");
}
*/
int MAIN(){
	int x=2147483647;
	int y=(-2147483647)-1;
	int a=1000;
	a=a+2147483647;
	a=a+9487;
	if(a>50){
		write("yes");
	}
	/*int y=(-2147483647)-1;*/
	if(x>=y){
		write("yes");
	}
	return 0;
}
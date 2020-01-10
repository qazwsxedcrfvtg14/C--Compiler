/*int n;
int test(int b,int c,int g,int e){
	int a,d;
	a = b * c + g * e;
	d = b * c + g + e;
	return a+d;
}

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
}*/

int True(){
	write("1");
	return 1;
}
int False(){
	write("0");
	return 0;
}

int MAIN(){
	int ar[100],x;
	int a;
	x=87;
	x=read();
	write(x);
	write("\n");
	ar[5]=10;
	ar[x]=60;
	write(ar[5]);
	write("\n");
	write(ar[x]);
	write("\n");
	a=True()&&True();
	write(" ");write(a);write("\n");
	a=True()&&False();
	write(" ");write(a);write("\n");
	a=False()&&True();
	write(" ");write(a);write("\n");
	a=False()&&False();
	write(" ");write(a);write("\n");
	a=True()||True();
	write(" ");write(a);write("\n");
	a=True()||False();
	write(" ");write(a);write("\n");
	a=False()||True();
	write(" ");write(a);write("\n");
	a=False()||False();
	write(" ");write(a);write("\n");
	return 0;
}
/*
int MAIN()

{

	int result;
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
	write("The factorial is ");
	write(result);
	write("\n");
}
*/
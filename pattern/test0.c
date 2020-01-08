int n;
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
}

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

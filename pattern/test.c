int n;
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
    int a0,b0,c0,d0,e0,f0,g0,h0,i0,j0,k0,l0,m0,o0,p0,q0,r0,s0,t0,u0,v0,w0,x0,y0,z0;
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
    z0=888;
	write("The factorial is ");
	write(result);
	write("\n");
}

int MAIN()
{
 int a,b,c,d,e,f,g;
 int xd[100000];
 int xd1[100000];
 int xd2[100000];
 int xd3[100000];
 int xd4[100000];
 a = read();
 b = read();
 c = a + 2; 
 d = b + 4;
 e = a + 1;
 f = b;
 g = c*d - 1; 
 
 if (a || b || c && d && (e>1) || (f>1)&&(g<1) )
  { 
    write("True \n");
  }
 else
  {
    write("False \n");
  }

   
 return 0;
}

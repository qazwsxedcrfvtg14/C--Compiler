TARGET = parser
OBJECT = parser.tab.c parser.tab.o lex.yy.c alloc.o functions.o semanticAnalysis.o symbolTable.o codegen.o
OUTPUT = parser.output parser.tab.h output.S output0.S
#CC = clang -g
#CXX = clang++ -g -std=c++17 -fsanitize=undefined
CC = gcc -g
CXX = g++ -g -O2 -std=c++17
LEX = flex
YACC = bison -v
YACCFLAG = -d
LIBS = -lfl -lstdc++


parser: parser.tab.o alloc.o functions.o symbolTable.o semanticAnalysis.o codegen.o
	$(CC) -o $(TARGET) parser.tab.o alloc.o functions.o symbolTable.o semanticAnalysis.o codegen.o $(LIBS)

parser.tab.o: parser.tab.c lex.yy.c alloc.o functions.c symbolTable.o semanticAnalysis.o
	$(CC) -c parser.tab.c
    
semanticAnalysis.o: semanticAnalysis.c symbolTable.o
	$(CC) -c semanticAnalysis.c

symbolTable.o: symbolTable.c
	$(CC) -c symbolTable.c

lex.yy.c: lexer3.l
	$(LEX) lexer3.l

parser.tab.c: parser.y 
	$(YACC) $(YACCFLAG) parser.y

alloc.o: alloc.c
	$(CC) -c alloc.c
	
functions.o: functions.c
	$(CC) -c functions.c

codegen.o: codegen.h codegen.cpp
	$(CXX) -c codegen.cpp

clean:
	rm -f $(TARGET) $(OBJECT) $(OUTPUT)


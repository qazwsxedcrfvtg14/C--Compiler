int true(){
    return 1;
}
int false(){
    return 0;
}
int MAIN() {
    int a, b;
    float c, d;
    a = 1;
    b = 0;
    if (true()+true()) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (true()&&true()) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (false()&&true()) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (true()&&false()) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (true()||true()) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (true()||false()) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (false()||true()) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (false()||false()) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (a > b) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (a < b) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (a >= b) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (a <= b) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (a-1 >= b) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (a <= b+1) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (a == b) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (a != b) {
        write("correct\n");
    } else {
        write("wrong\n");
    }

    c = 1.0;
    d = 0.0;
    if (c > d) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (c < d) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (c >= d) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (c <= d) {
        write("wrong\n");
    } else {
        write("correct\n");
    }
    if (c != d) {
        write("correct\n");
    } else {
        write("wrong\n");
    }
    if (c == d) {
        write("wrong\n");
    } else {
        write("correct\n");
    }

    if (a && b) {
        write("wrong\n");
    }

    if (a || b) {
        write("correct\n");
    }

    if (c && d) {
        write("wrong\n");
    }

    if (c || d) {
        write("correct\n");
    }
    c=c+d;
    c=c-d;
    c=c*d;
    c=c/d;
    a=b+a;
    a=b-a;
    a=b*a;
    a=b/a;

    a = 0;
    while (a<10) {
        write("correct: ");
        write(a);
        write("\n");
        a = a+1;
    }
    if (a == 10) {
        write("correct\n");
    }

    return 0;
}

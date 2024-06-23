#include <iostream>

int main() {
    int start = 0;
    int end = 500;
    int fr = 6;
    int cr = 5;

    for (int j = 0; j < fr; j++) {
        int sp = ((start) * (fr -j) + (end) *j)/fr;
        int ep = ((start + cr) * (fr -j-1) + (end) *(j+1))/fr -1;
        std::cout << sp << ", " << ep << std::endl;
    }
}
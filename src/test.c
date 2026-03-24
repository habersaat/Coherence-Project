// test.c
volatile int shared = 0;

int main() {
    while (1) {
        shared = shared + 1;
    }
}
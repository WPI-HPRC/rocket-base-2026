#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_state_machine();
void test_logging();

int main(int argc, char **argv) {
    UNITY_BEGIN();

    test_state_machine();
    test_logging();

    return UNITY_END();
}

void test_state_machine() {
    
}

void test_logging() {

}
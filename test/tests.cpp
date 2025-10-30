#include <unity.h> 
#include "main.hpp"

void simple_test() {
    TEST_ASSERT_TRUE(true);
}

void run() {
    setup();

    for(int i = 0; i < 100; i++) {
        loop();
    }
}


int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(simple_test);
    RUN_TEST(run);

    UNITY_END();
}
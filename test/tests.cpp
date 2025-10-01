#include <unity.h> 

void simple_test() {
    TEST_ASSERT_TRUE(true);
}


int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(simple_test);

    UNITY_END();
}
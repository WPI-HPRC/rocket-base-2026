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

// State Machine

void test_state_sanity() {
    TEST_ASSERT_TRUE(true);
}

void test_state_machine() {
    RUN_TEST(test_state_sanity);
}

// Logging

void test_logging_sanity() {
    TEST_ASSERT_TRUE(true);
}

void test_logging() {
    RUN_TEST(test_logging_sanity);
}
#include <Arduino.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_state_machine();
void test_logging();

void setup() {
    UNITY_BEGIN();

    test_state_machine();
    test_logging();

    UNITY_END();
}

void loop() {}

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
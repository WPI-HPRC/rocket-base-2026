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

void test_state_machine() {
    
}

void test_logging() {

}
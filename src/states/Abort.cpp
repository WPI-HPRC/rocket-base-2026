#include <State.hpp> // include/State/hpp

void abortInit (StateData* data) {}

State abortLoop (StateData *data, Context *ctx) {
    return ABORT; // this is what code from last year was doing, may need to do more
}
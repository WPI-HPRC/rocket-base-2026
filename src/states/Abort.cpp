#include "../State.h"

void abortInit (StateData* data) {}

StateID abortLoop (StateData *data, Context *ctx) {
    return ABORT; // this is what code from last year was doing, may need to do more
}
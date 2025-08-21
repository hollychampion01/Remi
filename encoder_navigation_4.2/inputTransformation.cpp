#include "inputTransformation.hpp"

using namespace mtrn3100;

void InputTrans::inputToCommand(const String& comm) {
    for (size_t i = 0; i < comm.length(); ++i) {
        char c = comm.charAt(i);
        if (c == 'l') {
            moveLeft();
        } else if (c == 'r') {
            moveRight();
        } else if (c == 'f') {
            moveFront();
        } else {
            Serial.print("Unknown command: ");
            Serial.println(c);
        }
    }
}

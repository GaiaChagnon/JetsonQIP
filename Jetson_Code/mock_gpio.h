#ifndef MOCK_GPIO_H
#define MOCK_GPIO_H

// Mock GPIO definitions for compilation testing
#define JET_INPUT  0
#define JET_OUTPUT 1

// Mock GPIO functions
inline int gpioInitialise() { return 0; }
inline void gpioTerminate() {}
inline int gpioSetMode(unsigned pin, unsigned mode) { (void)pin; (void)mode; return 0; }
inline int gpioWrite(unsigned pin, unsigned level) { (void)pin; (void)level; return 0; }
inline int gpioRead(unsigned pin) { (void)pin; return 0; }

#endif // MOCK_GPIO_H 
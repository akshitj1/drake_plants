#include "hello.h"
#include <iostream>

void say_hello(){
  std::cout << "Hello, world!!!" << std::endl;
}

int main(int argc, char const *argv[]) {
  say_hello();
  return 0;
}

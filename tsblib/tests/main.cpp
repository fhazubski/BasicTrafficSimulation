#include "include/tslib/api.h"

int main() {
  for (int i = 0; i < 200; i++) {
    tspSimulateKnospe(30, 30, 1.5, 1.5, 0.94, 0.5, 0.1, 6, 4.5, 1.5, 10.5, 2,
                      1000, 0.3, 500);
  }
  return 0;
}

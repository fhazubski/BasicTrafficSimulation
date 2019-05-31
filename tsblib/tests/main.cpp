#include "include/tslib/api.h"

int main() {
  for (int i = 0; i < 20; i++) {
    tspSimulateKnospe(37.5, 37.5, 2.5, 2.5, 0.5, 0.2, 0.1, 2, 5, 0.25, 1, 2,
                      1000, 0.3, 1000);
  }
  return 0;
}

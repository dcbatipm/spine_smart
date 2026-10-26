#pragma once
#include <stdint.h>
#include <math.h>

// Pure, clock-injected posture state machine, also exercised by on-device selftest.
class PostureLogic {
public:
  float threshold = 15.0f;
  uint32_t holdMs = 10000;
  bool pending = false;
  bool slouch = false;
  uint32_t since = 0;
  void reset() { pending = false; slouch = false; since = 0; }
  bool update(float angle, bool validAndStill, uint32_t now) {
    if (!validAndStill || !isfinite(angle)) { reset(); return false; }
    if (angle < threshold - 3.0f) { reset(); return false; }
    if (!pending) {
      if (angle < threshold) return false;
      pending = true; since = now;
    }
    if (!slouch && uint32_t(now - since) >= holdMs) { slouch = true; return true; }
    return false;
  }
};

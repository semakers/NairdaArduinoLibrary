#include "nairda_string.h"

int stringCompare(String a, String b) {
  return a.equals(b) ? 1 : 0;
}

String stringToLower(String s) {
  s.toLowerCase();
  return s;
}

String stringToUpper(String s) {
  s.toUpperCase();
  return s;
}

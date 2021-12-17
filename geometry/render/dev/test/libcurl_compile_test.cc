#include <curl/curl.h>

int main(int /* argc */, char* /* argv */[]) {
  CURL* curl;
  curl = curl_easy_init();
  curl_easy_cleanup(curl);
  // Test is to ensure curl compiles / runs, not that curl works.
  return 0;
}

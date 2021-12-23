#include <string>
#include <vector>

#include <picosha2.h>

int main(int /* argc */, char* /* argv */[]) {
  std::string data = "This is a test.";
  std::vector<unsigned char> hash(picosha2::k_digest_size);
  picosha2::hash256(data.begin(), data.end(), hash.begin(), hash.end());
  std::string sha = picosha2::bytes_to_hex_string(hash.begin(), hash.end());

  // If the sha is correct, we desire a return code of 0.
  // The literal hash value here can be checked via the command:
  //   echo -n "This is a test." | sha256sum
  return sha != "a8a2f6ebe286697c527eb35a58b5539532e9b3ae3b64d4eb0a46fb657b41562c";
}

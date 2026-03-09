#include "drake/geometry/render_gltf_client/internal_http_service_curl.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <thread>
#include <utility>
#include <vector>

#include <curl/curl.h>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace {

namespace fs = std::filesystem;

// Curl callbacks --------------------------------------------------------------

// Writes callback for libcurl, assumes `userp` points to an std::ofstream.
// See: https://curl.se/libcurl/c/CURLOPT_WRITEFUNCTION.html
size_t WriteFileData(void* buffer, size_t size, size_t nmemb, void* userp) {
  const size_t data_size = size * nmemb;
  std::ofstream* f = static_cast<std::ofstream*>(userp);
  f->write(static_cast<char*>(buffer), data_size);
  // Return 0 to indicate a failed write operation to libcurl.
  if (!f) return 0;
  return data_size;
}

using DebugData =
    std::vector<std::pair<const curl_infotype, const std::string>>;
/* Used with CURLOPT_VERBOSE in order to add to drake::log(), see also:
 https://curl.se/libcurl/c/CURLOPT_DEBUGFUNCTION.html

 `userp` is assumed to be DebugData defined above, this method simply
 accumulates the incoming messages from curl, LogCurlDebugData below combines
 and adds them to the drake log.  The following types are skipped, as these
 items should be handled by the write / read callback (binary data should not be
 logged to drake::log()):

 - CURLINFO_DATA_IN
 - CURLINFO_DATA_OUT
 - CURLINFO_SSL_DATA_IN
 - CURLINFO_SSL_DATA_OUT */
int DebugCallback(CURL* /* handle */, curl_infotype type, char* data,
                  size_t size, void* userp) {
  if (type != CURLINFO_DATA_IN && type != CURLINFO_DATA_OUT &&
      type != CURLINFO_SSL_DATA_IN && type != CURLINFO_SSL_DATA_OUT) {
    DebugData* debug_data = static_cast<DebugData*>(userp);
    debug_data->emplace_back(type, std::string(data, data + size));
  }

  return 0;  // See docs, return of 0 is always required.
}

// Used in logging curl data, returns a human friendly string.
std::string CurlInfoTypeAsString(curl_infotype type) {
  if (type == CURLINFO_TEXT) {
    return "CURLINFO_TEXT";
  } else if (type == CURLINFO_HEADER_IN) {
    return "CURLINFO_HEADER_IN";
  } else if (type == CURLINFO_HEADER_OUT) {
    return "CURLINFO_HEADER_OUT";
  } else if (type == CURLINFO_DATA_IN) {
    return "CURLINFO_DATA_IN";
  } else if (type == CURLINFO_DATA_OUT) {
    return "CURLINFO_DATA_OUT";
  } else if (type == CURLINFO_SSL_DATA_IN) {
    return "CURLINFO_SSL_DATA_IN";
  } else if (type == CURLINFO_SSL_DATA_OUT) {
    return "CURLINFO_SSL_DATA_OUT";
  } else if (type == CURLINFO_END) {
    return "CURLINFO_END";
  }
  return fmt::format("UNKNOWN_CURLINFO_TYPE={}", fmt_streamed(type));
}

/* Removes leading / trailing whitespace from `message` before logging.  Curl
 includes newline characters in its entries, which are desirable for the
 accumulated interior message parts but the trailing whitespace in particular
 leads to cluttered logs (drake::log() adds a trailing newline). */
void LogIfTrimmedWhitespaceNonEmpty(curl_infotype type,
                                    const std::string& message) {
  const auto trimmed =
      std::regex_replace(message, std::regex(R"(^\s+|\s+$)"), "");
  // Some curl messages were just the \n character, don't log "nothing".
  if (!trimmed.empty()) {
    drake::log()->debug("[{}] {}", CurlInfoTypeAsString(type), trimmed);
  }
}

/* Logs the combined messages added to `debug_data` by the DebugCallback.
 When curl calls the DebugCallback, it will do so with piecemeal components.  In
 the below walk-through, the "Accept: ..." statement was modified to add spaces
 between * and / to avoid ending the comment.

 Call 1: type=CURLINFO_HEADER_OUT
         data="  Trying 127.0.0.1:8000...\n"
 Call 2: type=CURLINFO_HEADER_OUT
         data="TCP_NODELAY set\n"
 Call 3: type=CURLINFO_HEADER_OUT
         data="Connected to 127.0.0.1 (127.0.0.1) port 8000 (#0)\n"
 Call 4: type=CURLINFO_TEXT
         data="POST /render HTTP/1.1
         Host: 127.0.0.1:8000
         Accept: * / *
         Content-Length: 1448
         Content-Type: multipart/form-data; boundary=--...---c13a866cb4280bc1\n"
 Call 5: type=CURLINFO_HEADER_IN
         data="We are completely uploaded and fine\n"
 ...

 Unlike after CURLINFO_HEADER_OUT, the CURLINFO_TEXT included before
 CURLINFO_HEADER_IN is provided one line at a time.  As a result, the
 drake::log() if added to directly in DebugCallback can become somewhat
 cluttered and inconsistent.  This method combines any sequential messages that
 were appended into a single accumulated log call, trimming preceding and
 trailing whitespace to keep the log as orderly as possible.  In the example
 above, the input vector would look something like:

 {
   {CURLINFO_HEADER_OUT, "  Trying 127.0.0.1:8000...\n"},
   {CURLINFO_HEADER_OUT, "TCP_NODELAY set\n"},
   {CURLINFO_HEADER_OUT, "Connected to 127.0.0.1 (127.0.0.1) port 8000 (#0)\n"},
   {CURLINFO_TEXT, "POST /render HTTP/1.1\n...\n...\n"},
   {CURLINFO_HEADER_IN, "We are completely uploaded and fine\n"},
 }

 and the drake::log() will have three total combined calls, the first three
 CURLINFO_HEADER_OUT will be combined into one log call, then CURLINFO_TEXT,
 then CURLINFO_HEADER_IN.
 */
void LogCurlDebugData(const DebugData& debug_data) {
  // NOTE: not very efficient in terms of copies, but messages are small.
  std::string accumulator{};
  /* The value must be initialized for compiler warnings, prev_type gets set on
   the first iteration. Initialize to intentionally invalid value. */
  curl_infotype prev_type = static_cast<curl_infotype>(-1);
  int counter = 0;
  for (const auto& pair : debug_data) {
    // On the first iteration, initialize the previous type and accumulator.
    if (counter++ == 0) {
      prev_type = pair.first;
      accumulator += pair.second;
    } else {
      /* If the current and previous type are the same, accumulate and continue.
       Otherwise, log the now complete accumulator and reset for the newly
       encountered curl_infotype. */
      if (pair.first == prev_type) {
        accumulator += pair.second;
      } else {
        LogIfTrimmedWhitespaceNonEmpty(pair.first, accumulator);
        prev_type = pair.first;
        accumulator = pair.second;
      }
    }
  }
  // Log final results (loop will have accumulated at least the last element).
  if (counter > 0) {
    LogIfTrimmedWhitespaceNonEmpty(prev_type, accumulator);
  }
}

// Helper utilities ------------------------------------------------------------

/* Returns the next unique-per-process integer that can uniquely identify a
 temporary file to use in a potential curl transaction. */
int64_t NextTempId() {
  static drake::never_destroyed<std::atomic<int64_t>> global_temp_id;
  return ++(global_temp_id.access());
}

std::string NextTempFile() {
  /* Create e.g., {temp_directory()}/0000000000000000XYZ.curl
   NOTE: the maximum number of digits in a int64_t is 19. */
  return fmt::format("{:0>19}.curl", NextTempId());
}

}  // namespace

HttpServiceCurl::HttpServiceCurl() : HttpService() {
  /* libcurl should be initialized exactly once per process, this initialization
   is not thread-safe and must be done before potential threads using curl begin
   (e.g., threaded renderings).  See also: MakeRenderEngineGltfClient
   documentation in factory.h */
  static CURLcode ignored =
      curl_global_init(CURL_GLOBAL_ALL | CURL_GLOBAL_ACK_EINTR);
  unused(ignored);
}

HttpServiceCurl::~HttpServiceCurl() {}

HttpResponse HttpServiceCurl::DoPostForm(const std::string& temp_directory,
                                         const std::string& url,
                                         const DataFieldsMap& data_fields,
                                         const FileFieldsMap& file_fields,
                                         bool verbose) {
  // Create and fill out a <form> to POST.
  CURL* curl{nullptr};
  CURLcode result;
  curl = curl_easy_init();
  DRAKE_DEMAND(curl != nullptr);
  curl_mime* form{nullptr};
  struct curl_slist* headerlist{nullptr};
  form = curl_mime_init(curl);

  /* Defined to make cleanup easier before throwing any possible exceptions.
   Frees resources for `curl`, `form`, and `headerlist`. */
  auto cleanup_curl = [](CURL* c, curl_mime* f, curl_slist* h_list) {
    if (f != nullptr) curl_mime_free(f);
    if (h_list != nullptr) curl_slist_free_all(h_list);
    curl_easy_cleanup(c);
  };

  // Used when verbose, needed in scope for logging after curl_easy_perform.
  DebugData debug_data;
  if (verbose) {
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, &DebugCallback);
    curl_easy_setopt(curl, CURLOPT_DEBUGDATA, &debug_data);
  }

  // Setup the POST url.
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

  // Add all of the data fields.
  for (const auto& [field_name, field_data] : data_fields) {
    curl_mimepart* field = curl_mime_addpart(form);
    curl_mime_name(field, field_name.c_str());
    curl_mime_data(field, field_data.c_str(), CURL_ZERO_TERMINATED);
  }

  // Add all of the file fields.
  for (const auto& [field_name, field_data_pair] : file_fields) {
    // Add the file to the form.
    const auto& file_path = field_data_pair.first;
    curl_mimepart* field = curl_mime_addpart(form);
    curl_mime_name(field, field_name.c_str());
    curl_mime_filedata(field, file_path.c_str());

    // Set the mimetype if provided.
    const auto& file_mimetype = field_data_pair.second;
    if (file_mimetype.has_value()) {
      curl_mime_type(field, file_mimetype.value().c_str());
    }
  }

  curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

  // Disable 100-Continue.  See:
  // http://www.iandennismiller.com/posts/curl-http1-1-100-continue-and-multipartform-data-post.html
  headerlist = curl_slist_append(headerlist, "Expect:");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);

  /* We do not know if the server is going to respond with anything, and if it
   does it will be e.g., json or image file response.  Write directly to a file
   buffer within our temporary directory. */
  const auto temp_bin_out = fs::path(temp_directory) / NextTempFile();
  const std::string bin_out_path{temp_bin_out.string()};
  if (fs::exists(temp_bin_out)) {
    cleanup_curl(curl, form, headerlist);
    throw std::runtime_error(fmt::format(
        "RenderClient: refusing to overwrite temporary file '{}' that "
        "already exists, please cleanup temporary directory '{}'.",
        bin_out_path, temp_directory));
  }

  // Open the file for writing, pass it off to curl.
  std::ofstream bin_out(bin_out_path, std::ios::binary);
  if (!bin_out.good()) {
    cleanup_curl(curl, form, headerlist);
    throw std::runtime_error(fmt::format(
        "RenderClient: unable to open temporary file '{}'.", bin_out_path));
  }
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &WriteFileData);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &bin_out);

  // Perform the POST and drake::log() prior to any potential exceptions.
  result = curl_easy_perform(curl);
  if (!bin_out.good()) {
    cleanup_curl(curl, form, headerlist);
    throw std::runtime_error(fmt::format(
        "RenderClient: unable to wtite temporary file '{}'.", bin_out_path));
  }
  if (verbose) {
    LogCurlDebugData(debug_data);
  }

  // Populate the wrapper return struct.
  HttpResponse ret;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &ret.http_code);
  if (result != CURLE_OK) {
    ret.service_error_message = std::string(curl_easy_strerror(result));
  }

  // Cleanup the curl memory.
  cleanup_curl(curl, form, headerlist);

  // Close the file after the write callback is complete. Delete the file if
  // it's empty.
  bin_out.close();
  const bool server_gave_data_response = fs::file_size(temp_bin_out) > 0;
  if (server_gave_data_response) {
    ret.data_path = bin_out_path;
  } else {
    try {
      fs::remove(bin_out_path);
    } catch (const std::exception& e) {
      drake::log()->debug("HttpServiceCurl unable to delete '{}'. {}",
                          bin_out_path, e.what());
    }
  }

  return ret;
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake

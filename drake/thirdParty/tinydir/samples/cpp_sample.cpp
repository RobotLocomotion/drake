#include <iostream>
#include <stdexcept>
#include <string>

struct tinydir_dir;

class TinyDir {
public:
    TinyDir(const std::string& path);
    ~TinyDir();
    std::string BaseName() const;
private:
    tinydir_dir* dir;
};

#include <tinydir.h>

TinyDir::TinyDir(const std::string& path) : dir(new tinydir_dir) {
    if (tinydir_open(dir, path.c_str()) == -1) {
        throw std::invalid_argument{"path"};
    }
}

TinyDir::~TinyDir() {
    delete dir;
}

std::string TinyDir::BaseName() const {
    const std::string path{dir->path};
    auto lastSlash = path.find_last_of("/\\");
    if (lastSlash == std::string::npos) {
        return path;
    }
    return path.substr(lastSlash + 1);
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: cpp_sample filename\n";
        return 1;
    }
    TinyDir td{argv[1]};
    std::cout << "Basename is " << td.BaseName() << "\n";
    return 0;
}


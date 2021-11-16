import sys


def main():
    with open(sys.argv[1], encoding="utf-8") as f:
        text = f.read()
    print(text)
    lines = text.splitlines()
    assert "Copyright" in lines[0]
    assert "SOFTWARE" in lines[-1]


assert __name__ == "__main__"
main()

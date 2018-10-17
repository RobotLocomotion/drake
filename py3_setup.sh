py3-setup() { (
    set -x
    python3 -m virtualenv --python python3 build/py3 --system-site-packages
    set +x
    source build/py3/bin/activate
    set -x
    rm build/py3/bin/python
    mv build/py3/bin/{python-config,python3-config}
    sed -i 's#py3/bin/python\b#py3/bin/python3#g' build/py3/bin/python3-config
    pip install -I \
        protobuf==3.6.0 sphinx==1.8.1 sphinx_rtd_theme
) }

if [[ ! -f build/py3/bin/python3 ]]; then
    py3-setup
fi

source build/py3/bin/activate
_python_bin=$(which python3)

bazel-py3() { (
    set -eu
    cmd=${1}
    shift
    set -x
    bazel ${cmd} \
        --python_path=${_python_bin} \
        --action_env=PYTHON_BIN_PATH=${_python_bin} \
        "$@"
) }

complete -F _bazel__complete -o nospace "bazel-py3"

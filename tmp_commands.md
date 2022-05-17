```sh
bazel build \
    //tools/release_engineering:download_release_candidate \
    //tools/release_engineering:relnotes

bazel-bin/tools/release_engineering/download_release_candidate \
    --version v1.3.0 --timestamp 20220517

commit=7fcdd44b8a48c2ae5b99ab60db31740f8192acc8
git merge ${commit}

bazel-bin/tools/release_engineering/relnotes --action=update \
    --version=v1.3.0 --target_commit=${commit}
```

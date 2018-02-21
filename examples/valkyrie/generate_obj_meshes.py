import os
import subprocess

filter_script = os.path.join(os.path.dirname(__file__), "fix_uv.mlx")

for (dirpath, dirnames, filenames) in os.walk(os.curdir):
    print(dirpath)
    for file in filenames:
        if file.endswith(".dae"):
            # meshlabserver crashes frequently and seemingly at random.
            # The horrifying workaround is to just try the same conversion
            # several times.
            for i in range(10):
                try:
                    subprocess.check_call(["meshlabserver", "-i", os.path.join(dirpath, file),
                                           "-s", filter_script,
                                           "-o", os.path.join(dirpath, file.replace(".dae", ".obj")),
                                           "-om", "vt", "vn", "wt"])
                    break
                except:
                    pass
            else:
                throw(ValueError("Couldn't convert mesh: {:s}".format(file)))

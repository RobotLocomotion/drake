"""
Example to show migration pain point for Anzu, from drake#14401;
use this to figure out next step for Anzu.

pre-#14401:  5839ca8992
post-#14401: 2d70dadf3b
"""
from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import GetScopedFrameByName, Parser
from pydrake.multibody.plant import MultibodyPlant

ARBITRARY_TIME_STEP = 0.01


def try_get_frame(plant, name):
    expr = f"GetScopedFrameByName(plant, {repr(name)})"
    try:
        result = str(GetScopedFrameByName(plant, name))
    except RuntimeError as e:
        result = f"ERROR: : {str(e)}"
    print(f"{expr}:\n  {result}")


def without_rename(model_file):
    plant = MultibodyPlant(time_step=ARBITRARY_TIME_STEP)
    Parser(plant).AddModelFromFile(model_file)
    # First worked pre-#14401 b/c this was unique.
    # But no longer; however, not extremely surprising.
    try_get_frame(plant, "dummy")
    # This always works, as expected.
    try_get_frame(plant, "dummy::dummy")


def with_rename(model_file):
    plant = MultibodyPlant(time_step=ARBITRARY_TIME_STEP)
    Parser(plant).AddModelFromFile(model_file, model_name="my_model")
    # This doesn't work post-#14401?
    try_get_frame(plant, "dummy")
    # This always works, as expected.
    try_get_frame(plant, "my_model::dummy")


def main():
    model_file = FindResourceOrThrow("drake/tmp/dummy.sdf")

    print("without_rename")
    without_rename(model_file)
    print()
    print("with_rename")
    with_rename(model_file)


if __name__ == "__main__":
    main()


"""
Command: bazel run //tmp:repro

Output pre-#14401:

    without_rename
    GetScopedFrameByName(plant, 'dummy'):
      <BodyFrame_[float] name='dummy' index=1 model_instance=2>
    GetScopedFrameByName(plant, 'dummy::dummy'):
      <BodyFrame_[float] name='dummy' index=1 model_instance=2>

    with_rename
    GetScopedFrameByName(plant, 'dummy'):
      <BodyFrame_[float] name='dummy' index=1 model_instance=2>
    GetScopedFrameByName(plant, 'my_model::dummy'):
      <BodyFrame_[float] name='dummy' index=1 model_instance=2>

post-#14401:

    without_rename
    GetScopedFrameByName(plant, 'dummy'):
      ERROR: : Frame dummy appears in multiple model instances.
    GetScopedFrameByName(plant, 'dummy::dummy'):
      <BodyFrame_[float] name='dummy' index=1 model_instance=2>

    with_rename
    GetScopedFrameByName(plant, 'dummy'):
      ERROR: : Frame dummy appears in multiple model instances.
    GetScopedFrameByName(plant, 'my_model::dummy'):
      <BodyFrame_[float] name='dummy' index=1 model_instance=2>
"""

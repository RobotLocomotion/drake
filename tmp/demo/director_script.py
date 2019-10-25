def set_camera_pos(focal_pt, camera_pos):
    c = view.camera()
    c.SetViewUp((0, 0, 1))
    c.SetFocalPoint(focal_pt)
    c.SetPosition(camera_pos)
    view.render()


def get_camera_pos_code():
    c = view.camera()
    print("")
    print("Code in `director_script.py`:")
    print("")
    print("set_camera_pos(\n   focal_pt={},\n   camera_pos={},\n)".format(
        repr(c.GetFocalPoint()), repr(c.GetPosition())))
    print("")


assert "view" in globals()

set_camera_pos(
   focal_pt=(-0.12157765992526606, -0.05447064902572543, 0.5868276965040243),
   camera_pos=(2.074305346841314, 2.14141235774086, 1.2455925985339995),
)

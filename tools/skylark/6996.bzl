# -*- python -*-

# TODO(jwnimmer-tri) This whole file and all of its functions should be purged
# once the #6996 transition is complete.  This TODO also covers the need to
# change all call sites to stop calling these functions.  The typical fix will
# be to just remove the function, leaving the unadulturated string or list that
# it was wrapping in its place.

# Whether or not the project-wide `git mv` for #6996 has happened yet.
HAS_MOVED_6996 = True

def adjust_labels_for_drake_hoist(labels):
    """Change //drake/foo/bar into //foo/bar, returning a new labels list."""
    if not labels:
        return []
    if type(labels) == "select":
        return labels
    result = []
    for one_label in labels:
        if HAS_MOVED_6996 and one_label.startswith("//drake/"):
            one_label = "//" + one_label[len("//drake/"):]
        elif HAS_MOVED_6996 and one_label.startswith("//drake:"):
            one_label = "//:" + one_label[len("//drake:"):]
        elif HAS_MOVED_6996 and one_label.startswith("@drake//drake/"):
            one_label = "@drake//" + one_label[len("@drake//drake/"):]
        result.append(one_label)
    return result

def adjust_label_for_drake_hoist(label):
    """Change //drake/foo/bar into //foo/bar, returning a new label."""
    return adjust_labels_for_drake_hoist([label])[0]

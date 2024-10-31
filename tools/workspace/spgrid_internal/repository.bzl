def spgrid_internal_repository(name):
    native.local_repository(
        name = name,
        path = "../../../third_party/spgrid/",
    )

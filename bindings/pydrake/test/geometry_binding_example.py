# -*- coding: utf8 -*-

import numpy as np
import pydrake
from pydrake.rbtree import RigidBodyTree

if __name__ == "__main__":
    rbtree = RigidBodyTree("Pendulum.urdf")

    for body_i in range(rbtree.get_num_bodies()):
        body = rbtree.get_body(body_i)
        visual_elements = body.get_visual_elements()
        this_body_patches = []
        for element in visual_elements:
            if element.hasGeometry():
                geom = element.getGeometry()
                print geom
                print geom.getShape()
                print geom.getPoints()
                print geom.getBoundingBoxPoints()
                if geom.hasFaces():
                    facesOut = np.ndarray((3,1))
                    print element.getGeometry().getFaces()
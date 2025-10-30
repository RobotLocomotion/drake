import pydrake.planning as mut  # ruff: isort: skip

import unittest

from pydrake.multibody.plant import MultibodyPlant


class TestDofMask(unittest.TestCase):
    def test_dof_set(self):
        plant = MultibodyPlant(time_step=0.0)
        instance = plant.AddModelInstance("asdf")
        plant.Finalize()

        empty = []
        self.assertEqual(
            mut.DofMask.MakeFromModel(plant=plant, model_index=instance), empty
        )
        self.assertEqual(
            mut.DofMask.MakeFromModel(plant=plant, model_name="asdf"), empty
        )

        empty_dofs = mut.DofMask()

        a = [False, True]
        b = [True, False]

        # Explicit constructors.
        self.assertEqual(mut.DofMask(values=a), a)
        for bool_val in [True, False]:
            self.assertEqual(
                mut.DofMask(size=3, value=bool_val), [bool_val] * 3
            )

        self.assertEqual(mut.DofMask(a).Complement(), [True, False])
        self.assertEqual(mut.DofMask(a).Union(other=b), [True, True])
        self.assertEqual(mut.DofMask(a).Intersect(other=b), [False, False])
        self.assertEqual(mut.DofMask(a).Subtract(other=b), [False, True])

        self.assertEqual(empty_dofs.GetJoints(plant=plant), empty)

        self.assertEqual(mut.DofMask([True, False]).size(), 2)
        self.assertEqual(mut.DofMask([True, False]).count(), 1)
        self.assertTrue(mut.DofMask([True, False])[0])
        self.assertFalse(mut.DofMask([True, False])[1])
        self.assertEqual(
            mut.DofMask([True, False]).GetFullToSelectedIndex(), [0, None]
        )
        self.assertEqual(
            mut.DofMask([False, True]).GetSelectedToFullIndex(), [1]
        )

import os
import numpy as np
import transforms3d as tf3d

from typing import Optional

from armarx import arviz as viz
from armarx import remote_gui as rg

from armarx.remote_gui.widgets.ndarray import NdArrayWidget


class Data:

    def __init__(self):
        super().__init__()

        self.linear_joints = np.zeros(2)
        self.eef_pos = np.zeros(3)

        self.links = np.array([
            [0, 0, 5],
            [0, 0, 5],
        ])

        self.fk()

    def fk(self):
        # Dummy.
        if False:
            eef = np.zeros(3)

            eef += tf3d.axangles.axangle2mat((1, 0, 0), self.linear_joints[0]) @ self.links[0]
            eef += tf3d.axangles.axangle2mat((1, 0, 0), self.linear_joints[1]) @ self.links[1]

            self.eef_pos = eef
        self.eef_pos[:] = [result[PE_xs], result[PE_ys], result[PE_zs]]

    def ik(self):
        pass


class Visu:

    def __init__(self, data: Data):
        self.data = data

    def visualize(self, stage: viz.Stage):
        self.vis_linear_joints(stage.layer("Linear Joints"))
        self.vis_eef_pos(stage.layer("EEF Pos"))
        stage.origin_layer(scale=1e-3)

    def vis_linear_joints(self, layer: viz.Layer):
        ground_radius = 5
        for i, value in enumerate(self.data.linear_joints):
            angle = i * np.pi/2
            start = ground_radius * np.array([np.cos(angle), np.sin(angle), 0])
            if value == 0:
                value = 1e-3
            end = start + value * 10 * np.array([0, 0, 1])
            layer.add(viz.Arrow(f"linear joint {i}", from_to=(start, end),
                                width=0.2, color=(255, 192, 0)))

    def vis_eef_pos(self, layer: viz.Layer):
        layer.add(viz.Sphere("eef pos", position=self.data.eef_pos, radius=1, color=(0, 0, 255)))
        layer.add(viz.Pose("eef pose", position=self.data.eef_pos, scale=1e-3))


class WidgetsTab(rg.Tab):

    def __init__(
            self,
            data: Data,
            visu: Visu,
            tag=None,
            arviz: Optional[viz.Client] = None):
        super().__init__(tag or TAG)

        self.data = data
        self.visu = visu
        self.arviz = arviz or viz.Client(tag)

        self.linear_joints = NdArrayWidget(data.linear_joints, row_vector=True, range_min=0, range_max=10)
        self.eef_pos = NdArrayWidget(data.eef_pos, row_vector=True, range_min=-100, range_max=100)

    def create_widget_tree(self) -> rg.GridLayout:
        layout = rg.GridLayout()
        row = 0

        layout.add(rg.Label("Linear Joints:"), (row, 0))
        layout.add(self.linear_joints.create_tree(), (row, 1))
        row += 1

        layout.add(rg.Label("EEF Pos:"), (row, 0))
        layout.add(self.eef_pos.create_tree(), (row, 1))
        row += 1

        return layout

    def update(self):

        if self.linear_joints.has_any_changed():
            self.data.linear_joints = self.linear_joints.get_array()
            self.data.fk()

        if self.eef_pos.has_any_changed():
            self.data.eef_pos = self.eef_pos.get_array()
            self.data.ik()

        with self.arviz.begin_stage(commit_on_exit=True) as stage:
            self.visu.visualize(stage)


if __name__ == "__main__":
    TAG = os.path.basename(__file__)

    data = Data()
    visu = Visu(data)
    arviz = viz.Client(TAG)

    with arviz.begin_stage(commit_on_exit=True) as stage:
        visu.visualize(stage)

    rg_client = rg.Client()

    widgets_tab = WidgetsTab(data=data, visu=visu, tag=TAG)
    rg_client.add_tab(widgets_tab)
    rg_client.receive_updates()
    rg_client.receive_updates()

    rg_client.update_loop(lambda: widgets_tab.update(), block=False)

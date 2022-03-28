import os
import numpy as np
import transforms3d as tf3d
import typing as ty

from armarx import arviz as viz
from armarx import remote_gui as rg

from armarx.remote_gui.widgets.ndarray import NdArrayWidget


class Data:

    def __init__(self):
        super().__init__()

        self.linear_joints = np.zeros(2)
        self.eef_pos = np.zeros(3)
        self.eef_ori = np.identity(3)

        self.links = np.array([
            [0, 0, 5],
            [0, 0, 5],
        ])

        self.fk()

    def fk(self):
        print("-" * 50)
        pos = self.eef_pos.copy()
        ori = self.eef_ori.copy()

        # Dummy.
        do_revolute = False
        if do_revolute:
            pos = np.zeros(3)
            pos += tf3d.axangles.axangle2mat((1, 0, 0), self.linear_joints[0]) @ self.links[0]
            pos += tf3d.axangles.axangle2mat((1, 0, 0), self.linear_joints[1]) @ self.links[1]

        # KIT-Wrist constants
        lever = 1
        theta_0 = np.deg2rad(25)
        print(f"(a1, a2) = {self.linear_joints}")

        do_azim_zenith = False
        if do_azim_zenith:
            from armarx.math import spherical
            from .equations import f_azimuth, f_zenith

            azimuth = f_azimuth(*self.linear_joints, L=lever, T_0=theta_0)
            zenith = f_zenith(*self.linear_joints, L=lever, T_0=theta_0)
            radius = lever

            elevation = np.deg2rad(90) - zenith
            pos = spherical.spherical2cartesian([radius, azimuth, elevation])
            print(f"(azimuth, zenith) = ({azimuth:.3f}, {zenith:.3f}) | Cartesian = {np.round(pos, 3)}")

        do_fk = True
        if do_fk:
            from .equations import fk, fk_ori

            pos = fk(*self.linear_joints, L=lever, T_0=theta_0)
            ori = fk_ori(pos[0], pos[1], L=lever, T_0=theta_0)
            print(f"(x, y, z) = {np.round(pos, 3)}")

        self.eef_pos = pos
        self.eef_ori = ori

    def ik(self):
        pass


class Visu:

    def __init__(self, data: Data):
        self.data = data

    def visualize(self, stage: viz.Stage):
        self.vis_linear_joints(stage.layer("Linear Joints"))
        self.vis_eef_pos(stage.layer("EEF Pos"))

        stage.origin_layer(scale=3e-3)


    def vis_linear_joints(self, layer: viz.Layer):
        ground_radius = 1.5
        for i, value in enumerate(self.data.linear_joints):
            angle = i * np.pi/2
            start = ground_radius * np.array([np.cos(angle), np.sin(angle), 0])
            if value == 0:
                value = 1e-3
            end = start + value * 1 * np.array([0, 0, 1])
            layer.add(viz.Arrow(f"linear joint {i}", from_to=(start, end),
                                width=0.05, color=(255, 192, 0)))

        layer.add(viz.Cylinder("circle", from_to=((0, 0, 0), (0, 0, 1e-3)),
                               radius=1, color=(255, 255, 0, 255)))
        layer.add(viz.Sphere("sphere", radius=1, color=(255, 255, 0, 64)))

    def vis_eef_pos(self, layer: viz.Layer):
        pos = self.data.eef_pos
        layer.add(viz.Sphere("eef pos", position=pos, radius=0.05, color=(0, 0, 255)))
        layer.add(viz.Pose("eef pose", position=pos, orientation=self.data.eef_ori, scale=3e-3))
        layer.add(viz.Arrow("eef vector", from_to=((0, 0, 0), pos),
                            width=0.02, color=(30, 30, 70)))


class WidgetsTab(rg.Tab):

    def __init__(
            self,
            data: Data,
            visu: Visu,
            tag=None,
            arviz: ty.Optional[viz.Client] = None):
        super().__init__(tag or TAG)

        self.data = data
        self.visu = visu
        self.arviz = arviz or viz.Client(tag)

        limit = 0.7
        self.linear_joints = NdArrayWidget(data.linear_joints, row_vector=True,
                                           range_min=-limit, range_max=limit)
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

        try:
            with self.arviz.begin_stage(commit_on_exit=True) as stage:
                self.visu.visualize(stage)
        except np.linalg.LinAlgError as e:
            print(e)


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

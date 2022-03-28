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

        self.actuator_pos = np.zeros(2)
        self.actuator_vel = np.zeros(2)

        self.eef_pos = np.zeros(3)
        self.eef_ori = np.identity(3)

        self.eef_vel = np.zeros(3)
        self.eef_rot_vel = np.zeros(3)

        self.fk()

    def fk(self):
        print("-" * 50)
        a1, a2 = self.actuator_pos

        # KIT-Wrist constants
        lever = 1
        theta_0 = np.deg2rad(25)
        print(f"(a1, a2) = {self.actuator_pos}")

        do_azim_zenith = False
        if do_azim_zenith:
            from armarx.math import spherical
            from hemisphere_joint_demo.equations import f_azimuth, f_zenith

            azimuth = f_azimuth(*self.actuator_pos, L=lever, T_0=theta_0)
            zenith = f_zenith(*self.actuator_pos, L=lever, T_0=theta_0)
            radius = lever

            elevation = np.deg2rad(90) - zenith
            pos = spherical.spherical2cartesian([radius, azimuth, elevation])
            print(f"(azimuth, zenith) = ({azimuth:.3f}, {zenith:.3f}) | Cartesian = {np.round(pos, 3)}")
            self.eef_pos = pos

        do_fk = True
        if do_fk:
            from hemisphere_joint_demo.equations import fk, fk_ori

            pos = fk(*self.actuator_pos, L=lever, T_0=theta_0)
            ori = fk_ori(pos[0], pos[1], L=lever, T_0=theta_0)

            self.eef_pos = pos
            self.eef_ori = ori

        do_jac = True
        if do_jac:
            from hemisphere_joint_demo.equations import jacobian
            jac = jacobian(a1=a1, a2=a2, L=lever, T_0=theta_0)
            print(f"Jacobian: \n{np.round(jac, 3)}")

            vel = jac @ self.actuator_vel
            self.eef_vel = vel[:3]
            self.eef_rot_vel = vel[3:]

        print(f"EEF pos = {np.round(self.eef_pos, 3)} (norm = {np.linalg.norm(self.eef_pos):.3f})")
        print(f"EEF vel = {np.round(self.eef_vel, 3)} (norm = {np.linalg.norm(self.eef_vel):.3f})")

        print(f"EEF ori = \n{np.round(self.eef_ori, 3)}")
        print(f"EEF rot vel = {np.round(self.eef_rot_vel, 3)} (norm = {np.linalg.norm(self.eef_rot_vel):.3f})")

    def ik(self):
        pass


class Visu:

    def __init__(self, data: Data):
        self.data = data

    def visualize(self, stage: viz.Stage):
        self.vis_actuators(stage.layer("Actuators"))
        self.vis_eef(stage.layer("EEF Pos"))
        self.vis_sphere(stage.layer("Sphere"))

        stage.origin_layer(scale=3e-3)

    def vis_actuators(self, layer: viz.Layer):
        def angle_to_pos(angle):
            return ground_radius * np.array([np.cos(angle), np.sin(angle), 0])

        ground_radius = 1.5
        z = np.array([0, 0, 1])
        for i, (pos, vel) in enumerate(zip(self.data.actuator_pos, self.data.actuator_vel)):
            # Position
            if pos == 0:
                pos = 1e-3
            angle = i * np.pi/2
            start = angle_to_pos(angle)
            end = start + pos * 1 * z
            layer.add(viz.Arrow(f"actuator pos {i}", from_to=(start, end),
                                width=0.05, color=(255, 192, 0)))

            # Velocity
            if vel == 0:
                vel = 1e-3
            angle += 1/32 * 2*np.pi
            start = angle_to_pos(angle)
            end = start + vel * 1 * z
            layer.add(viz.Arrow(f"actuator vel {i}", from_to=(start, end), width=0.02, color=(255, 32, 0)))

    def vis_eef(self, layer: viz.Layer):
        # Env
        pos = self.data.eef_pos
        vel = self.data.eef_vel

        layer.add(viz.Sphere("eef pos", position=pos, radius=0.05, color=(0, 0, 255)))
        layer.add(viz.Pose("eef pose", position=pos, orientation=self.data.eef_ori, scale=3e-3))
        layer.add(viz.Arrow("eef vector", from_to=((0, 0, 0), pos), width=0.02, color=(30, 30, 70)))

        if np.linalg.norm(vel) >= 1e-3:
            speed = 1.0
            vel_end = pos + speed * vel
            layer.add(viz.Arrow("eef velocity", from_to=(pos, vel_end), width=0.02, color=(0, 255, 70)))

    def vis_sphere(self, layer: viz.Layer):
        layer.add(viz.Cylinder("circle", from_to=((0, 0, 0), (0, 0, 1e-3)),
                               radius=1, color=(255, 255, 0, 255)))
        layer.add(viz.Sphere("sphere", radius=1, color=(255, 255, 0, 64)))




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
        self.actuator_pos = NdArrayWidget(data.actuator_pos, row_vector=True,
                                          range_min=-limit, range_max=limit)
        self.actuator_vel = NdArrayWidget(data.actuator_vel, row_vector=True,
                                          range_min=-limit, range_max=limit)

        self.eef_pos = NdArrayWidget(data.eef_pos, row_vector=True, range_min=-100, range_max=100)
        self.eef_vel = NdArrayWidget(data.eef_vel, row_vector=True, range_min=-100, range_max=100)

    def create_widget_tree(self) -> rg.GridLayout:
        layout = rg.GridLayout()
        row = 0

        layout.add(rg.Label("Actuator Pos:"), (row, 0)).add(self.actuator_pos.create_tree(), (row, 1))
        row += 1
        layout.add(rg.Label("Actuator Vel:"), (row, 0)).add(self.actuator_vel.create_tree(), (row, 1))
        row += 1

        layout.add(rg.Label("EEF Pos:"), (row, 0)).add(self.eef_pos.create_tree(), (row, 1))
        row += 1
        layout.add(rg.Label("EEF Vel:"), (row, 0)).add(self.eef_vel.create_tree(), (row, 1))
        row += 1

        return layout

    def update(self):

        if self.actuator_pos.has_any_changed() or self.actuator_vel.has_any_changed():
            self.data.actuator_pos = self.actuator_pos.get_array()
            self.data.actuator_vel = self.actuator_vel.get_array()
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


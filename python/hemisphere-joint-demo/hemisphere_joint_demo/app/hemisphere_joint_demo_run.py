import enum
import os
import numpy as np
import typing as ty

from armarx import arviz as viz
from armarx import remote_gui as rg

from armarx.remote_gui.widgets.ndarray import NdArrayWidget


def spherical2cartesian(
        spherical: ty.Union[ty.List, np.ndarray],
) -> np.ndarray:
    spherical = np.array(spherical)

    assert spherical.shape[-1] == 3
    radius = spherical[..., 0]
    azim = spherical[..., 1]
    elev = spherical[..., 2]
    inclination = np.pi / 2 - elev
    sin_inclination = np.sin(inclination)

    cartesian = spherical.astype(np.float).copy()
    cartesian[..., 0] = radius * sin_inclination * np.cos(azim)  # x
    cartesian[..., 1] = radius * sin_inclination * np.sin(azim)  # y
    cartesian[..., 2] = radius * np.cos(inclination)  # z
    return cartesian


class Data:
    class Mode(enum.IntEnum):
        FK = 0
        IK = 1

    def __init__(self):
        super().__init__()

        self.lever = 1
        self.theta_0 = np.deg2rad(25)
        self.radius = 2 * np.sin(self.theta_0) * self.lever
        self.actuator_offset = np.arcsin(self.theta_0)

        self.mode = self.Mode.FK

        self.actuator_pos = np.zeros(2)
        # self.actuator_pos[:] = (0.5, 0.5)
        self.actuator_vel = np.zeros(2)

        self.eef_pos = np.zeros(3)
        self.eef_ori = np.identity(3)

        self.eef_vel = np.zeros(3)
        self.eef_rot_vel = np.zeros(3)

        self.fk()

    def fk(self):
        print("-" * 50)
        a1, a2 = self.actuator_pos + self.actuator_offset

        # KIT-Wrist constants
        lever = self.lever
        theta_0 = self.theta_0
        radius = self.radius
        print(f"Lever: {lever}")
        print(f"theta0: {theta_0}")
        print(f"radius: {radius}")
        print(f"actuator_offset: {self.actuator_offset}")
        print(f"(a1, a2) = ({a1}, {a2})")

        do_fk_pos_azim_zenith = False
        if do_fk_pos_azim_zenith:
            from hemisphere_joint_demo.equations import f_azimuth, f_zenith

            azimuth = f_azimuth(a1=a1, a2=a2, L=lever, T_0=theta_0)
            zenith = f_zenith(a1=a1, a2=a2, L=lever, T_0=theta_0)

            elevation = np.deg2rad(90) - zenith
            self.eef_pos = spherical2cartesian([radius, azimuth, elevation])
            print(f"(azimuth, zenith) = ({azimuth:.3f}, {zenith:.3f})")

        do_fk = True
        if do_fk:
            from hemisphere_joint_demo.equations import fk_pos
            self.eef_pos = fk_pos(a1, a2, L=lever, T_0=theta_0)

        do_fk_ori = True
        if do_fk_ori:
            from hemisphere_joint_demo.equations import fk_ori
            self.eef_ori = fk_ori(self.eef_pos[0], self.eef_pos[1], L=lever, T_0=theta_0)

        do_jac = True
        if do_jac:
            from hemisphere_joint_demo.equations import jacobian
            jac = jacobian(a1=a1, a2=a2, L=lever, T_0=theta_0)
            print(f"Jacobian: \n{np.round(jac, 3).tolist()}")
            # print(f"Jacobian: \n{jac}")

            vel = jac @ self.actuator_vel
            self.eef_vel = vel[:3]
            self.eef_rot_vel = vel[3:]

        print(f"EEF pos = {np.round(self.eef_pos, 3)} (norm = {np.linalg.norm(self.eef_pos):.3f})")
        print(f"EEF vel = {np.round(self.eef_vel, 3)} (norm = {np.linalg.norm(self.eef_vel):.3f})")

        print(f"EEF ori = \n{np.round(self.eef_ori, 3)}")
        print(f"EEF rot vel = {np.round(self.eef_rot_vel, 3)} (norm = {np.linalg.norm(self.eef_rot_vel):.3f})")

    def ik(self):
        from hemisphere_joint_demo.equations import fk_pos, ik

        print("-" * 50)

        eef_pos = self.radius * self.eef_pos / np.linalg.norm(self.eef_pos)
        print(f"EEF pos = {np.round(eef_pos, 3)} (norm = {np.linalg.norm(eef_pos):.3f})")

        ex, ey, ez = eef_pos
        a1, a2 = ik(ex, ey, L=self.lever, T_0=self.theta_0)
        self.actuator_pos[:] = [a1, a2]

        fk_eef_pos = fk_pos(a1, a2, L=self.lever, T_0=self.theta_0)
        error = fk_eef_pos - eef_pos
        print(f"FK(a..) = {np.round(fk_eef_pos, 3)}")
        print(f"Error   = {np.round(error, 5)}, |E| = {np.linalg.norm(error):.5f}")

        print(f"Actuator pos = {self.actuator_pos}")


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
            angle = i * np.pi / 2
            start = angle_to_pos(angle)
            end = start + pos * 1 * z
            layer.add(viz.Arrow(f"actuator pos {i}", from_to=(start, end),
                                width=0.05, color=(255, 192, 0)))

            # Velocity
            if vel == 0:
                vel = 1e-3
            angle += 1 / 32 * 2 * np.pi
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
        layer.add(viz.Arrow("eef out vector", from_to=(pos, pos + .5 * self.data.eef_ori @ np.array([0, 0, 1])),
                            color=(0, 0, 255), width=0.02))

        if np.linalg.norm(vel) >= 1e-3:
            speed = 1.0
            vel_end = pos + speed * vel
            layer.add(viz.Arrow("eef velocity", from_to=(pos, vel_end), width=0.02, color=(0, 255, 70)))

    def vis_sphere(self, layer: viz.Layer):
        layer.add(viz.Cylinder("circle", from_to=((0, 0, 0), (0, 0, 1e-3)),
                               radius=1, color=(255, 255, 0, 255)))
        layer.add(viz.Sphere("sphere", radius=self.data.radius, color=(255, 255, 0, 64)))


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

        self.mode = rg.ComboBox(options=[m.name for m in Data.Mode])

        limit = 0.7
        self.actuator_pos = NdArrayWidget(data.actuator_pos, row_vector=True,
                                          range_min=-limit, range_max=limit)
        self.actuator_vel = NdArrayWidget(data.actuator_vel, row_vector=True,
                                          range_min=-limit, range_max=limit)

        self.eef_pos = NdArrayWidget(data.eef_pos, row_vector=True, range_min=-self.data.radius, range_max=self.data.radius)
        self.eef_vel = NdArrayWidget(data.eef_vel, row_vector=True, range_min=-1, range_max=1)

    def create_widget_tree(self) -> rg.GridLayout:
        layout = rg.GridLayout()
        row = 0

        layout.add(rg.Label("Mode:"), (row, 0)).add(self.mode, (row, 1))
        row += 1

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
        updated = False

        if self.mode.has_value_changed():
            self.data.mode = Data.Mode[self.mode.value]
            updated = True

        if self.data.mode == Data.Mode.FK:
            if updated or self.actuator_pos.has_any_changed() or self.actuator_vel.has_any_changed():
                self.data.actuator_pos = self.actuator_pos.get_array()
                self.data.actuator_vel = self.actuator_vel.get_array()
                self.data.fk()
                updated = True

        if self.data.mode == Data.Mode.IK:
            if updated or self.eef_pos.has_any_changed():
                self.data.eef_pos = self.eef_pos.get_array()
                self.data.ik()
                updated = True

        if updated:
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

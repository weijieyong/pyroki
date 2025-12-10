"""IK with Manipulability

Inverse Kinematics with Manipulability using PyRoKi.
Modified for RM65 robot arm.

NOTE: Uses the working pks.solve_ik as base and adds manipulability visualization only.
The manipulability_cost in the optimizer causes issues with certain URDF configurations.
"""

import time
import viser
import yourdfpy
import numpy as np
import sys

sys.path.insert(0, 'examples')
import pyroki_snippets as pks

import pyroki as pk
from viser.extras import ViserUrdf


def main():
    """Main function for basic IK with manipulability visualization."""

    urdf_path = "/home/simt-wj/05_urdf/rm_65_b/urdf/rm_65_b_base_dummy_gripper.urdf"
    urdf = yourdfpy.URDF.load(urdf_path)
    target_link_name = "dummy_tcp"

    # Create robot.
    robot = pk.Robot.from_urdf(urdf)
    
    print(f"Robot links: {robot.links.names}")
    print(f"Robot joints: {robot.joints.names}")
    print(f"Num actuated joints: {robot.joints.num_actuated_joints}")
    print(f"Target link index: {robot.links.names.index(target_link_name)}")

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # Create interactive controller with initial position
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(0.5, 0.0, 0.5), wxyz=(0, 0, 1, 0)
    )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)
    value_handle = server.gui.add_number("Yoshikawa Index", 0.001, disabled=True)
    
    # Manipulability ellipse visualization
    manip_ellipse = pk.viewer.ManipulabilityEllipse(
        server,
        robot,
        root_node_name="/manipulability",
        target_link_name=target_link_name,
    )

    print("Starting IK loop...")
    iteration = 0
    while True:
        # Solve IK using the working basic solver
        start_time = time.time()
        solution = pks.solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=np.array(ik_target.position),
            target_wxyz=np.array(ik_target.wxyz),
        )
        
        # Debug output every 100 iterations
        # if iteration % 100 == 0:
        #     print(f"Iteration {iteration}: target_pos={ik_target.position}, solution={solution}")

        # Update manipulability visualization
        manip_ellipse.update(solution)
        value_handle.value = manip_ellipse.manipulability

        # Update timing handle.
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # Update visualizer.
        urdf_vis.update_cfg(solution)
        iteration += 1


if __name__ == "__main__":
    main()

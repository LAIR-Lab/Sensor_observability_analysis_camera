import numpy as np
import PyKDL as kdl
from kdyKDLl_parser_py.urdf import treeFromString


class CameraChain:
    def __init__(self, urdf_xml: str, base_link: str, camera_link: str, full_n_joints: int):
        ok, tree = treeFromString(urdf_xml)
        if not ok:
            raise RuntimeError("Failed to parse URDF into KDL tree")

        self.chain = tree.getChain(base_link, camera_link)
        self.n = self.chain.getNrOfJoints()
        if self.n > full_n_joints:
            raise RuntimeError(
                f"Chain to '{camera_link}' has {self.n} joints, "
                f"more than full_n_joints={full_n_joints}. Check joint_names/order."
            )

        self.full_n = full_n_joints
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

    def _to_jntarray(self, q_local: np.ndarray) -> kdl.JntArray:
        q_kdl = kdl.JntArray(self.n)
        for i in range(self.n):
            q_kdl[i] = float(q_local[i])
        return q_kdl

    def fk(self, q_full: np.ndarray) -> kdl.Frame:
        q_kdl = self._to_jntarray(q_full[: self.n])
        frame = kdl.Frame()
        self.fk_solver.JntToCart(q_kdl, frame)
        return frame

    def jacobian_full(self, q_full: np.ndarray) -> np.ndarray:
        """Returns a 6 x full_n_joints Jacobian, zero-padded past this chain's joint count."""
        q_kdl = self._to_jntarray(q_full[: self.n])
        jac = kdl.Jacobian(self.n)
        self.jac_solver.JntToJac(q_kdl, jac)

        J_local = np.array([[jac[r, c] for c in range(self.n)] for r in range(6)])
        J_full = np.zeros((6, self.full_n))
        J_full[:, : self.n] = J_local
        return J_full
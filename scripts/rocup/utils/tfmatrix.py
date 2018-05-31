import math
import numpy as np
import PyKDL
import tf
import random


class TfMatrix(object):

    def __init__(self, base=PyKDL.Frame(), size=np.array([1, 1, 1]), size_m=np.array([0.05, 0.05, 0.05])):
        self.size = size
        self.size_n = size_m
        self.base = base
        self.names = []
        self.transforms = []
        self.frames_map = {}
        self.frames_names = []
        self.frame_index = -1

    def resetIndex(self):
        self.frame_index = -1

    def isEmpty(self):
        index = self.frame_index + 1
        return index >= len(self.frames_names)

    def pickNextFrame(self):
        self.frame_index += 1
        if self.frame_index >= len(self.frames_names):
            raise IndexError('Frames names is over!')
        name = self.frames_names[self.frame_index]
        return (
            name,
            self.frames_map[name]
        )


class TfMatrixCube(TfMatrix):

    def __init__(self, base=PyKDL.Frame(), size=np.array([1, 1, 1]), size_m=np.array([0.05, 0.05, 0.05]), relative_frames=True, execution_order=[0, 1, 2], name_matrix=""):
        super(TfMatrixCube, self).__init__(base=base, size=size, size_m=size_m)

        a_lb = range(0, self.size[0])
        b_lb = range(0, self.size[1])
        c_lb = range(0, self.size[2])

        n = self.size[0] * self.size[1] * self.size[2]
        c_list = c_lb
        a_list = []
        b_list = []

        for b in range(0, self.size[1] * self.size[2]):
            a_list.extend(a_lb)
            a_lb.reverse()

        for c in range(0, self.size[2]):
            b_list.extend(b_lb)
            b_lb.reverse()

        indices_list = []
        for s in range(0, n):
            a = s
            b = int(s / self.size[0])
            c = int(s / (self.size[0] * self.size[1]))

            index = (
                a_list[a],
                b_list[b],
                c_list[c]
            )
            indices_list.append(index)

        for index in indices_list:
            x = index[execution_order[0]]
            y = index[execution_order[1]]
            z = index[execution_order[2]]
            dx = size_m[0] * x
            dy = size_m[1] * y
            dz = size_m[2] * z

            if name_matrix != "":
                name = "_" + name_matrix
            else:
                name = ""
            current_name = "matrix_cube_{}_{}_{}{}".format(x, y, z, name)

            if relative_frames:
                current_frame = self.base * PyKDL.Frame(PyKDL.Vector(dx, dy, dz))
            else:
                current_frame = self.base + PyKDL.Frame(PyKDL.Vector(dx, dy, dz))

            self.frames_names.append(current_name)
            self.frames_map[current_name] = current_frame


class TfMatrixHyperCube(TfMatrixCube):
    def __init__(self, base=PyKDL.Frame(), size_p=np.array([1, 1, 1]), size_r=np.array([1, 1, 1]), size_m_p=np.array([0.05, 0.05, 0.05]), size_m_r=np.array([0.0, 0.0, 0.0]), relative_frames=True, p_execution_order=[0, 1, 2], r_execution_order=[0, 1, 2]):
        super(TfMatrixHyperCube, self).__init__(base=base, size=size_p, size_m=size_m_p, relative_frames=relative_frames, execution_order=p_execution_order)

        self.size_r = size_r

        a_lb = range(0, size_r[0])
        b_lb = range(0, size_r[1])
        c_lb = range(0, size_r[2])

        n = size_r[0] * size_r[1] * size_r[2]
        c_list = c_lb
        a_list = []
        b_list = []

        for b in range(0, size_r[1] * size_r[2]):
            a_list.extend(a_lb)
            a_lb.reverse()

        for c in range(0, size_r[2]):
            b_list.extend(b_lb)
            b_lb.reverse()

        indices_list = []
        for s in range(0, n):
            a = s
            b = int(s / size_r[0])
            c = int(s / (size_r[0] * size_r[1]))

            index = (
                a_list[a],
                b_list[b],
                c_list[c]
            )
            indices_list.append(index)

        for frame_name in self.frames_map.keys():
            frame = self.frames_map.pop(frame_name)
            self.frames_names.remove(frame_name)

            for index in indices_list:
                ar = index[r_execution_order[0]]
                ap = index[r_execution_order[1]]
                ay = index[r_execution_order[2]]
                dar = size_m_r[0] * ar
                dap = size_m_r[1] * ap
                day = size_m_r[2] * ay
                current_name = "{}_{}_{}_{}".format(frame_name, ar, ap, ay)

                if relative_frames:
                    current_frame = frame * PyKDL.Frame(PyKDL.Rotation.RPY(dar, dap, day))
                else:
                    current_frame = frame + PyKDL.Frame(PyKDL.Rotation.RPY(dar, dap, day))

                self.frames_names.append(current_name)
                self.frames_map[current_name] = current_frame

    def order(self, mode="xyzrpy"):

        if mode == "xyzrpy":
            self.frames_names = sorted(self.frames_names, key=lambda frame: frame[-11])
            self.frames_names = sorted(self.frames_names, key=lambda frame: frame[-7])
            self.frames_names = sorted(self.frames_names, key=lambda frame: frame[-9])
            self.frames_names = sorted(self.frames_names, key=lambda frame: frame[-5])
            self.frames_names = sorted(self.frames_names, key=lambda frame: frame[-3])
            self.frames_names = sorted(self.frames_names, key=lambda frame: frame[-1])

            n = self.size[0] * self.size[1] * self.size[2]
            ir = n * self.size_r[0]
            ip = ir * self.size_r[1]
            iy = ip * self.size_r[2]

            for iblock in range(0, self.size_r[1] * self.size_r[2]):
                if iblock % 2 != 0:
                    self.frames_names[0 + iblock * ir:ir * (iblock + 1)] = sorted(self.frames_names[0 + iblock * ir:ir * (iblock + 1)], key=lambda frame: frame[-5], reverse=True)

            for iblock in range(0, self.size_r[2]):
                if iblock % 2 != 0:
                    self.frames_names[0 + iblock * ip:ip * (iblock + 1)] = sorted(self.frames_names[0 + iblock * ip:ip * (iblock + 1)], key=lambda frame: frame[-3], reverse=True)

        elif mode == "random":
            random.shuffle(self.frames_names)


class TfMatrixSphere(TfMatrix):

    def __init__(self, base=PyKDL.Frame(), size=np.array([1, 1, 1]), size_m=np.array([0.05, 0.05, 0.05]), indices_offset=np.array([0, 0, 1]), relative_frames=True, execution_order=[0, 1, 2], name_matrix=""):
        super(TfMatrixSphere, self).__init__(base=base, size=size, size_m=size_m)

        a_lb = range(0, self.size[0])
        b_lb = range(0, self.size[1])
        c_lb = range(0, self.size[2])

        n = self.size[0] * self.size[1] * self.size[2]
        c_list = c_lb
        a_list = []
        b_list = []

        for b in range(0, self.size[1] * self.size[2]):
            a_list.extend(a_lb)
            a_lb.reverse()

        for c in range(0, self.size[2]):
            b_list.extend(b_lb)
            b_lb.reverse()

        indices_list = []
        for s in range(0, n):
            a = s
            b = int(s / self.size[0])
            c = int(s / (self.size[0] * self.size[1]))

            index = (
                a_list[a],
                b_list[b],
                c_list[c]
            )
            indices_list.append(index)

        for index in indices_list:
            ithetaZ = index[execution_order[0]]
            ithetaY = index[execution_order[1]]
            iz = index[execution_order[2]]
            dthetaZ = size_m[0] * (ithetaZ + 1 + indices_offset[0])
            dthetaY = size_m[1] * (ithetaY + 1 + indices_offset[1])
            dz = size_m[2] * (iz + 1 + indices_offset[2])

            if name_matrix != "":
                name_matrix = "_" + name_matrix
            else:
                name = ""
            current_name = "matrix_sphere_{}_{}_{}{}".format(ithetaZ, ithetaY, iz, name)

            Tr_rz = PyKDL.Frame(PyKDL.Rotation().RotZ(dthetaZ))
            Tr_ry = PyKDL.Frame(PyKDL.Rotation().RotY(dthetaY))
            Tr_z = PyKDL.Frame(PyKDL.Vector(0, 0, dz))
            Tr = Tr_rz * Tr_ry * Tr_z

            if relative_frames:
                current_frame = self.base * Tr
            else:
                current_frame = self.base + Tr

            self.frames_names.append(current_name)
            self.frames_map[current_name] = current_frame

    def order(self, mode="reverse"):

        if mode == "reverse":
            self.frames_names.reverse()

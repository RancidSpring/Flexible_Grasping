import numpy as np

class FaceGeom:
    def __init__(self, normal_vec, face_center, face_index):
        self.normal_vec = normal_vec
        self.face_center = face_center
        self.end = normal_vec + face_center
        self.ind = face_index

    def __lt__(self, other):
        if np.linalg.norm(self.face_center) < np.linalg.norm(other.face_center):
            return True
        else:
            return False

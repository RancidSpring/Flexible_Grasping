

class FaceGeom:
    def __init__(self, normal_vec, face_center):
        self.normal_vec = normal_vec
        self.face_center = face_center
        self.end = normal_vec + face_center

def solve_head_pose(self, face_landmarks):
        indices = [17, 21, 22, 26, 36, 39, 42, 45, 31, 35]
        image_pts = np.zeros((len(indices), 2))
        for i in range(len(indices)):
            part = face_landmarks.part(indices[i])
            image_pts[i, 0] = part.x
            image_pts[i, 1] = part.y

        _, rotation_vec, translation_vec = cv2.solvePnP(self.face_model_points,
                                                        image_pts,
                                                        self.camera_matrix,
                                                        self.distortion_coeffs)
        projected_head_pose_box_points, _ = cv2.projectPoints(self.head_pose_box_points,
                                                              rotation_vec,
                                                              translation_vec,
                                                              self.camera_matrix,
                                                              self.distortion_coeffs)
        projected_head_pose_box_points = tuple(map(tuple, projected_head_pose_box_points.reshape(8, 2)))

        # Calculate euler angle
        rotation_mat, _ = cv2.Rodrigues(rotation_vec)
        pose_mat = cv2.hconcat((rotation_mat, translation_vec))
        _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
        return projected_head_pose_box_points, euler_angles 
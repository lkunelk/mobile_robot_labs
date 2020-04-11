import os
import glob

import numpy as np
import cv2
import scipy.io as sio
import matplotlib
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class FeatureProcessor:
    def __init__(self, data_folder, n_features=500, median_filt_multiplier=1.0):
        # Initiate ORB detector and the brute force matcher
        self.n_features = n_features
        self.median_filt_multiplier = median_filt_multiplier
        self.orb = cv2.ORB_create(nfeatures=n_features)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.data_folder = data_folder
        self.num_images = len(glob.glob(data_folder + '*.jpeg'))
        self.feature_match_locs = np.ones(
            (self.num_images, n_features, 2)) * -1  # [img_i, feat_i, [x, y] of match ((-1, -1) if no match)]

        # store the features found in the first image here. you may find it useful to store both the raw
        # keypoints in kp, and the numpy (u, v) pairs (keypoint.pt) in kp_np
        self.features = dict(kp=[], kp_np=[], des=[])
        self.first_matches = True
        return

    def get_image(self, id):
        # Load image and convert to grayscale
        img = cv2.imread(os.path.join(self.data_folder, 'camera_image_{}.jpeg'.format(id)))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def harris_features(self, id):
        gray = self.get_image(id)
        blockSize = 2
        ksize = 3
        k = 0.04
        dst = cv2.cornerHarris(gray, blockSize, ksize, k)
        return dst

    def get_features(self, id):
        """ Get the keypoints and the descriptors for features for the image with index id."""
        img = self.get_image(id)
        kp = self.orb.detect(img, None)
        key_points, descriptors = self.orb.compute(img, kp)

        # draw only keypoints location,not size and orientation
        # img2 = cv2.drawKeypoints(img, key_points, img, color=(255), flags=0)
        # plt.imshow(img2)
        # plt.show()

        return key_points, descriptors

    def append_matches(self, matches, new_kp):
        """ Take the current matches and the current keypoints
        and append them to the list of consistent match locations. """

    def get_matches(self):
        """ Get all of the locations of features matches for each image to the features found in the
        first image. Output should be a numpy array of shape (num_images, num_features_first_image, 2), where
        the actual data is the locations of each feature in each image."""

        img0 = self.get_image(0)
        key_points0, descriptors0 = self.get_features(0)
        self.features['kp'] = key_points0
        self.features['kp_np'] = [kp.pt for kp in key_points0]
        self.features['des'] = descriptors0

        for i in range(self.num_images):
            img = self.get_image(i)
            key_points, descriptors = self.get_features(i)
            matches = self.bf.match(descriptors0, descriptors)

            # add match locations
            for match in matches:
                kp = key_points[match.trainIdx]
                self.feature_match_locs[i][match.queryIdx] = kp.pt

            # draw matches
            # img3 = cv2.drawMatches(img0, key_points0, img, key_points, matches, None, flags=2)
            # plt.imshow(img3)
            # plt.show()


def triangulate(feature_data, tf, inv_K):
    """ For (u, v) image locations of a single feature for every image, as well as the corresponding
    robot poses as T matrices for every image, calculate an estimated xyz position of the feature.

    You're free to use whatever method you like, but we recommend a solution based on least squares, similar
    to section 7.1 of Szeliski's book "Computer Vision: Algorithms and Applications". """

    def compute_v_c(feature_loc, T):
        feature = np.append(feature_loc, 1).reshape(3, 1)
        R, c = T[:3, :3], T[:3, 3:4]
        v = R @ inv_K @ feature
        v = v / np.linalg.norm(v)
        return (v, c)

    cameras = []
    v_hats = []

    cvs = [compute_v_c(feature_data[i], tf[i]) for i in range(tf.shape[0]) if
           (feature_data[i] != np.array((-1, -1))).all()]

    # print(np.shape(cvs))

    def ransac(cvs):
        cvs = np.array(cvs)  # for list index
        # hyperparameters
        N = 3  # number of data to try
        tol = 0.01  # tolerance
        K = int(0.8 * len(cvs))  # threshold
        L = 3000  # max number of iterations
        # ransac
        for i in range(L):
            try:
                rand_idx = np.random.randint(len(cvs), size=N)
                sum_IvvT = sum([np.identity(3) - cv[0] @ cv[0].T for cv in cvs[rand_idx]])
                sum_IvvT_c = sum([(np.identity(3) - cv[0] @ cv[0].T) @ cv[1] for cv in cvs[rand_idx]])
                pc = np.linalg.inv(sum_IvvT) @ sum_IvvT_c
                # compute error
                r_sq = [np.linalg.norm((np.identity(3) - cv[0] @ cv[0].T) @ (pc - cv[1])) for cv in cvs]
                k = sum(np.array(r_sq) < tol)
                if k > K:
                    return pc.T
            except np.linalg.LinAlgError:
                continue  # singular matrix, this
        return np.ones(3)*0

    pc = ransac(cvs)

    return pc


def main():

    min_feature_views = 20  # minimum number of images a feature must be seen in to be considered useful
    K = np.array([[530.4669406576809, 0.0, 320.5],  # K from sim
                  [0.0, 530.4669406576809, 240.5],
                  [0.0, 0.0, 1.0]])
    inv_K = np.linalg.inv(K)  # will be useful for triangulating feature locations

    # load in data, get consistent feature locations
    data_folder = os.path.join(os.getcwd(), 'l3_mapping_data/')
    f_processor = FeatureProcessor(data_folder, n_features=100)
    f_processor.get_matches()  # output shape should be (num_images, num_features, 2)

    # feature rejection
    good_feature_locations = f_processor.feature_match_locs
    print(np.shape(good_feature_locations))
    feat_to_remove = []
    for feat_i in range(f_processor.n_features):
        count = 0
        for img_i in range(f_processor.num_images):
            if not good_feature_locations[img_i][feat_i][0] == -1:
                count+=1
        if count < 50:
            feat_to_remove.append(feat_i)

    for feat_i in reversed(feat_to_remove):
        good_feature_locations = np.delete(good_feature_locations, feat_i, 1)

    print(np.shape(good_feature_locations))
    num_landmarks = good_feature_locations.shape[1]

    pc = np.zeros((num_landmarks, 3))

    # create point cloud map of features
    tf = sio.loadmat("l3_mapping_data/tf.mat")['tf']
    tf_fixed = np.linalg.inv(tf[0, :, :]).dot(tf).transpose((1, 0, 2))
    for i in range(num_landmarks):
        # YOUR CODE HERE!! You need to populate good_feature_locations after you reject bad features!
        pc[i] = triangulate(good_feature_locations[:, i, :], tf_fixed, inv_K)

    # ------- PLOTTING TOOLS ------------------------------------------------------------------------------
    # you don't need to modify anything below here unless you change the variable names, or want to modify
    # the plots somehow.

    # get point cloud of trajectory for comparison
    traj_pc = tf_fixed[:, :3, 3]

    # view point clouds with matplotlib
    # set colors based on y positions of point cloud
    max_y = pc[:, 1].max()
    min_y = pc[:, 1].min()
    colors = np.ones((num_landmarks, 4))
    colors[:, :3] = .5
    colors[:, 1] = (pc[:, 1] - min_y) / (max_y - min_y)
    pc_fig = plt.figure()
    ax = pc_fig.add_subplot(111, projection='3d')
    ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], marker='^', color=colors, label='features')

    ax.scatter(traj_pc[:, 0], traj_pc[:, 1], traj_pc[:, 2], marker='o', label='robot poses')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.view_init(elev=-30, azim=-88)
    ax.legend()

    # fit a plane to the feature point cloud for illustration
    # see https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    centroid = pc.mean(axis=0)
    pc_minus_cent = pc - centroid  # subtract centroid
    u, s, vh = np.linalg.svd(pc_minus_cent.T)
    normal = u.T[2]

    # plot the plane
    # plane is ax + by + cz + d = 0, so z = (-ax - by - d) / c, normal is [a, b, c]
    # normal from svd, point is centroid, get d = -(ax + by + cz)
    d = -centroid.dot(normal)
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    X, Y = np.meshgrid(np.linspace(xlim[0], xlim[1], 10),
                       np.linspace(ylim[0], ylim[1], 10))
    Z = (-normal[0] * X - normal[1] * Y - d) * 1. / normal[2]
    ax.plot_wireframe(X, Y, Z, color='k')

    # view all final good features matched on first image (to compare to point cloud)
    feat_fig = plt.figure()
    ax = feat_fig.add_subplot(111)
    ax.imshow(f_processor.get_image(0), cmap='gray')
    ax.scatter(good_feature_locations[0, :, 0], good_feature_locations[0, :, 1], marker='^', color=colors)

    plt.interactive(True)
    plt.show()

    pc_fig.savefig('point_clouds.png', bbox_inches='tight')
    feat_fig.savefig('feat_fig.png', bbox_inches='tight')


def nam_test():
    data_folder = os.path.join(os.getcwd(), 'l3_mapping_data/')
    proc = FeatureProcessor(data_folder)
    proc.get_matches()


if __name__ == '__main__':
    main()

import numpy as np

#define the function to build the observability matrix from the sensor geometry
# This is for force torque sensors.
def build_observability_matrix(r, R):

    sensor_axes = [

        np.array([1.,0.,0.]),
        np.array([0.,1.,0.]),
        np.array([0.,0.,1.]),

        np.array([1.,0.,0.]),
        np.array([0.,1.,0.]),
        np.array([0.,0.,1.])

    ]

    columns = []

    for axis_local in sensor_axes:

        axis_world = R @ axis_local

        torque_component = np.cross(
            axis_world,
            r
        )

        column = np.concatenate([
            torque_component,
            axis_world
        ])

        columns.append(column)

    return np.array(columns).T

def observability_index(S):
#Row-wise Sum function.
    s = np.sum(
        np.abs(S),
        axis=1
    )

    return float(
        np.prod(s)
    )

def observability_index_p_norm(S):
#Row-wise P-Norm function.
    s = np.sum(
        np.abs(S),
        axis=1
    )

    return float(
        np.prod(s)
    )

def observability_index_max(S):
#Row-wise Max function.
    s = np.sum(
        np.max(S),
        axis=1
    )

    return float(
        np.prod(s)
    )

'''
From here the maths is for Camera-SOA.

'''

FOV = 1.047/2 # in radians, corresponds to 60 degrees

def build_observability_matrix_camera(r, R, fov=FOV):

    sensor_axes = [

        np.array([1.,0.,0.]),
        np.array([0.,1.,0.]),
        np.array([0.,0.,1.])

    ]

    columns = []

    for axis_local in sensor_axes:

        axis_world = R @ axis_local

        # For cameras, the observability is related to the angle between the sensor axis and the vector to the point.
        # We can use the dot product to find this angle.

        cos_angle = np.dot(axis_world, r) / (np.linalg.norm(axis_world) * np.linalg.norm(r))

        # The observability contribution from this sensor can be modeled as a function of the angle and the distance.
        # For example, we can use a simple model where the contribution is proportional to the cosine of the angle and inversely proportional to the distance.

        contribution = cos_angle / np.linalg.norm(r)

        columns.append(contribution)

    return [np.array(columns).reshape(-1, 1),cos_angle]

# Without Occlusion involved, the observability index for cameras can be defined as a function of the angle and distance to the point.
def observability_index_camera(S):
    # For cameras, we can define the observability index as the sum of the contributions from all sensors.
    s = S[0]
    theta = np.arccos(S[1])
    array=[0, (FOV-theta)/FOV]
    s = np.max(array, axis=0)
    return float(s)

def observability_index_camera_occlusion(S,S_,dist):
    s = S[0]
    theta = np.arccos(S[1])
    array=[0, (FOV-theta)/FOV]
    s = np.max(array, axis=0)
    return float(s/dist)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Program Title: kinematic_model.py
# Program Purpose: Define EEZYbotARM_kinematics parent class and EEZYbotARM_Mk2 child class

# **Version control**
# v3.1 -> adding save of np array for complex hull, adding complex hull function (this works in very basic form)
# v3.2 -> getting the complex hull function to pass data to a overall ploting function (it's also working)
# v3.3 -> troubleshooting why the convex hull and workspace isn't quite right! [19 Nov 19]
# v3.4 -> checking demo functionality
# v3.5 -> moving to github
# v3.6 -> adding MK1 version of the EEZYbotARM as a new sub-class
# v3.7 -> squishing bug for Mk1 inverse kinematics ----> Solved :) !

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# ------------------------------------------#
# Imports                                   #
# ------------------------------------------#

from math import *  # for maths
import matplotlib.pyplot as plt  # for plotting
from mpl_toolkits.mplot3d import Axes3D  # for 3D plotting
# [PUNCH LIST] NEED TO DO SOMETHING ABOUT THIS -> SAME AS ABOVE !
import mpl_toolkits.mplot3d as a3
import numpy as np  # for vector arithmetic
import pathlib  # for dealing with files
# [PUNCH LIST] needed? -> it's only used to generate a random number, I should be able to delete (pending)
import scipy as sp
# [PUNCH LIST] needed for plotting the complex hull -> I think i can do this in another way!
import pylab as pl
from scipy.spatial import ConvexHull
import pickle  # used for saving python files


# ------------------------------------------#
# Helper functions                          #
# ------------------------------------------#

def plotCoOrd(T0toN, ax, lineColor):
    """
    Plot a co-ordinate frame for a transformation matrix (in this case for a joint) using unit vectors.
    The plot is made using 3 'quiver objects' (arrows)

    --Parameters--
    @T0toN -> the co-ordinate frame to plot
    @ax -> the matplotlib 3d axes object to make the plot on
    @lineColor -> the color the co-ordinate frame

    """

    # Invert T0toN because we are rotating from the world frame to the N frame
    TNto0 = np.linalg.inv(T0toN)
    R = TNto0[0:3, 0:3]  # Find rotation matrix
    T_XYZ = T0toN[0:3, 3]  # Find translation vector

    # Unit Vector Positions before rotation for (x,y,x) positions and XYZ vectors (i)
    UV_XYZ = np.zeros((3, 3, 3), dtype=float)
    # Where dims are (start position of quiver, end position of quiver, respective xyz quivers)

    # define the unit vectors
    for i in range(0, 3):
        UV_XYZ[i, 1, i] = 50

    # define the co-ordinates to be plotted
    for i in range(0, 3):
        # make the rotation and translation
        UV_XYZ[:, 0, i] = R.dot(UV_XYZ[:, 0, i]) + T_XYZ[i]
        UV_XYZ[:, 1, i] = R.dot(UV_XYZ[:, 1, i]) + \
            T_XYZ[i]  # find end position (matlab)
        UV_XYZ[:, 2, i] = UV_XYZ[:, 1, i] - \
            UV_XYZ[:, 0, i]  # find end position (python)

    # these labels are used in the plot to label co-ordinate axes
    labels = ['X', 'Y', 'Z']

    # plot the x, y, z quivers
    for i in range(0, 3):

        # data for quiver
        x = UV_XYZ[i, 0, 0]  # label the co-ordinates
        y = UV_XYZ[i, 0, 1]
        z = UV_XYZ[i, 0, 2]
        u = UV_XYZ[i, 2, 0]
        v = UV_XYZ[i, 2, 1]
        w = UV_XYZ[i, 2, 2]

        # plot each quiver
        ax.quiver(x, y, z, u, v, w, length=80, normalize=True,
                  color=lineColor, linewidth=0.5)

        # data for text
        labels = ['x', 'y', 'z']
        zdir = None

        # plot text label
        ax.text(x+u, y+v, z+w, labels[i], zdir, color='gray')


# These helper functions help set axes equal in 3d
# Unaltered code from https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
def set_axes_radius(ax, origin, radius):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    set_axes_radius(ax, origin, radius)


def in_hull(p, hull):
    """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed


    --Example of use--

    tested = np.random.rand(20,3)
    cloud  = np.random.rand(50,3)

    print in_hull(tested,cloud) -> [True, True, False, etc.]

    """
    from scipy.spatial import Delaunay
    if not isinstance(hull, Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p) >= 0


# ------------------------------------------#
# Parent class                              #
# ------------------------------------------#

class EEZYbotARM:
    """
    --Description--
    This is a parent class for the EExybot Robotic arm

    **Please note that you must always instantiate on a child class (either EEZYbotARMMk2() or EEZYbotARMMk1()) 
    rather than instansiating the parent 'EEZYbotARM' class**

    --Methods--
    Description of available methods in this class:    
        - __init__ --> Initialise a robotic arm with current joint position
        - updateJointAngles --> Update the stored joint angles for the robot arm
        - checkErrorJointLimits --> Check if any of the supplied joint angles are outside of physical limits
        - FK_EEZYbotARM --> Perform forward kinematics to find the end effector position, given joint angles
        - IK_EEZYbotARM --> Perform inverse kinematics to find the joint angles, given end effector position
        - plot_EEZYbotARM --> Plot the robotic arm in 3D using matplotlib Axes 3D

    """

    # Class attribute
    # e.g. species = 'mammal'

    # Initializer / Instance attributes
    def __init__(self, initial_q1, initial_q2, initial_q3):
        self.q1 = initial_q1
        self.q2 = initial_q2
        self.q3 = initial_q3

    # Instance methods
    def updateJointAngles(self, q1, q2, q3):
        """
        --Description--
        Update the three EEzybot Arm joint angles (by providing values in degrees)
        Will return an error message if the joint angles are outside of given limits

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees)
        @q2 -> the value of the angle q2 (in degrees) 
        @q3 -> the value of the angle q3 (in degrees)

        --Returns--
        Function doesn't return a value

        """
        # Check given values are inside valid limits
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)

        # Assign new angles
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def checkErrorJointLimits(self, **kwargs):
        """
        --Description--
        Take the three angles for the EzzyBot Arm defined in kinematics.
        Use these to calculate whether the EzzyBot Arm is outside of joint limits
        If outside of joint limts return error code for appropriate joint
        Otherwise return zero value

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        servoAngle_q1, servoAngle_q2, servoAngle_q3 -> values in degrees for output to the physical servos

        """
        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Find angle limits
        q3_min, q3_max = self.q3CalcLimits(q2=q2)

        # Check given angles are within limits, else raise exception
        if q1 < self.q1_min or q1 > self.q1_max:
            raise Exception('Value for q1 is outside joint limits. Joint limits in degrees are ({},{})'.format(
                self.q1_min, self.q1_max))

        if q2 < self.q2_min or q2 > self.q2_max:
            raise Exception('Value for q2 is outside joint limits. Joint limits in degrees are ({},{})'.format(
                self.q2_min, self.q2_max))

        if q3 < q3_min or q3 > q3_max:
            raise Exception(
                'Value for q3 is outside joint limits. Joint limits in degrees are ({},{})'.format(q3_min, q3_max))

    def forwardKinematics(self, **kwargs):
        """
        --Description--
        Given angles q1, q2, q3, find the forward kinematics for the end effector position of the EEZYbotARM

        If q1, q2, q3 are not provided then the current arm angles will be used (self.q1,q2,q3)

        The forward kinematics uses DH proximal convention and the method is disclosed seperate to this code

        The reference angle definitions are as follows:

        - EzzyBot base (q1) : 0 degree position is facing directly forwards
        - Main arm position (q2)
        - Horarm position(q3)

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        x_EE, y_EE, z_EE -> position of the end effector in the world frame (mm)

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Convert to radians
        q1 = q1 * pi/180
        q2 = q2 * pi/180
        q3 = q3 * pi/180

        # Joint length definitions (for ease of reading code)
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Find the position of the end effector using the forward kinematics equations
        x_EE = round((cos(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*cos(q1)), 3)
        y_EE = round((sin(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*sin(q1)), 3)
        z_EE = round((L1 + sin(q2)*L2 + sin(q2+q3)*L3), 3)

        # Return the end effector position in (mm)
        return x_EE, y_EE, z_EE

    def inverseKinematics(self, x_EE, y_EE, z_EE):
        """
        --Description--
        Function: to find the Inverse Kinematics for the EEZYbotARM Mk2 

        Description: given x y z positions of the desired EE position inside the
        manipulator workspace, find the corresponding joint angles

        --Parameters--
        x_EE, y_EE, z_EE -> the cartesian position of the end effector in the world frame

        --Returns--
        q1, q2, q3 -> Corresponding joint angles in degrees

        """
        # DH parameters (Proximal Convention)
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Find the value for the fist angle
        q1 = atan2(y_EE, x_EE)

        # Find the values for the position of joint #4 for x, y, z
        x_4 = x_EE - (L4 * cos(q1))
        y_4 = y_EE - (L4 * sin(q1))
        z_4 = z_EE

        # Find the length of the third side of the (virtual) triangle made by L2,
        # L3 in the vertical plane of the robot arm

        # --- Specify the z poisition of joint #1
        z_1 = L1

        # --- First we find the z distance between joint #1 and joint #4 (in the world frame)
        z_1_4 = z_4 - z_1

        # --- Find the true distance (in x, y plane) to joint #4
        xy_4 = sqrt((x_4**2)+(y_4**2))

        # --- Then we find the length of the virtual side made by the right angle triangle
        v_side = sqrt((z_1_4**2) + (xy_4**2))

        # Find the value for the angle at joint #3
        q3 = - (pi - acos((L2**2 + L3**2 - v_side**2)/(2 * L2 * L3)))

        # Find the value for the angle at joint #2 %DEBUG HERE
        # --- Find the value of the angle from the x-y plane to the virtual side
        q2_a = atan2(z_1_4, xy_4)

        q2_b = acos((v_side**2 + L2**2 - L3**2)/(2 * v_side * L2))

        q2 = q2_a + q2_b  # NOTE there's some more work to do here to make this correctly summation or subtraction dependant on the position of the arm!

        # Print the input world frame position of the end effector
        print('Input Position of End Effector: \n')
        print('x_EE: {}'.format(x_EE))
        print('y_EE: {}'.format(y_EE))
        print('z_EE: {} \n'.format(z_EE))

        # Print the output joint angles
        print('Ouput joint angles: \n')
        print('q1: {:+.2f}'.format(q1 * 180/pi))
        print('q2: {:+.2f}'.format(q2 * 180/pi))
        print('q3: {:+.2f} \n'.format(q3 * 180/pi))

        # round values
        q1 = round(q1 * 180/pi, 2)
        q2 = round(q2 * 180/pi, 2)
        q3 = round(q3 * 180/pi, 2)

        return q1, q2, q3

    def plot(self, **kwargs):
        """
        --Description--
        Given angles q1, q2, q3, plot EEZybot arm and co-ordinate frames for all joints using matplotlib
        Function computes the configuration of the manipulator and plots it using DH proximal convention.

        The function will raise an exception if any of the angles are outside defined limits

        The reference angle definitions are as follows:

        - EzzyBot base (q1) : 0 degree position is facing directly forwards
        - Main arm position (q2)
        - Horarm position(q3)

        --Required--
        Helper functions required (included in the EEzybot code)
        - plotCoOrd()
        - set_axes_radius()
        - set_axes_equal()

        --Optional **kwargs parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        fig, ax -> matplotlib figure and axes objects showing position of all actuator joints and links in world frame

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Convert angles to radians
        q1 = q1 * pi/180
        q2 = q2 * pi/180
        q3 = q3 * pi/180

        # DH parameters (Proximal Convention),
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4
        # Lengths for the links which attach to the hoarm
        L2A = self.L2A
        LAB = self.LAB
        LB3 = self.LB3

        # DH table
        DH = np.array([[0,  0,     L1, q1],
                       [0,  pi/2,  0,  q2],
                       [L2, 0,     0,  q3],
                       [L3, 0,     0,  0],
                       [0,  -pi/2, 0,  0]])

        # Find number of rows in DH table
        rows, cols = DH.shape

        # Pre-allocate Array to store Transformation matrix
        T = np.zeros((4, 4, rows), dtype=float)

        # Determine transformation matrix between each frame
        for i in range(rows):
            T[:, :, i] = [[cos(DH[i, 3]),              -sin(DH[i, 3]),               0,             DH[i, 0]],
                          [sin(DH[i, 3])*cos(DH[i, 1]),  cos(DH[i, 3]) *
                           cos(DH[i, 1]), -sin(DH[i, 1]), -sin(DH[i, 1])*DH[i, 2]],
                          [sin(DH[i, 3])*sin(DH[i, 1]),  cos(DH[i, 3]) *
                           sin(DH[i, 1]),  cos(DH[i, 1]),  cos(DH[i, 1])*DH[i, 2]],
                          [0,                          0,                          0,             1]]

        # Create the transformation frames with repect to the world frame (the base of the EEzybot arm)

        # --- Calculate Transformation matrix to each frame wrt the base. (matrix dot multiplication)
        T00 = np.identity(4)
        T01 = T[:, :, 0]
        T02 = T[:, :, 0].dot(T[:, :, 1])
        T03 = T[:, :, 0].dot(T[:, :, 1]).dot(T[:, :, 2])
        T04 = T[:, :, 0].dot(T[:, :, 1]).dot(T[:, :, 2]).dot(T[:, :, 3])

        # --- Create frame 5 (note this is just a rotation of frame T04)
        R5 = T04[0:3, 0:3]  # Find rotation matrix
        T45 = np.zeros((4, 4))
        T45[0:3, 0:3] = np.linalg.inv(R5)
        T45[3, 3] = 1  # Create transformation matrix from frame 4 to frame 5

        # Create the transformation matrix from the world frame to frame 5 (without z rotation)
        T05 = T04.dot(T45)

        # --- Construct a transformation matrix to make the Z rotation of T05 by magnitude q1
        TZRot = np.array([[cos(q1),  -sin(q1), 0, 0],
                          [sin(q1),   cos(q1), 0, 0],
                          [0,         0,       1, 0],
                          [0,         0,       0, 1]])

        # Create the transformation matrix from the world frame to frame 5 (with z rotation)
        T05_true = T05.dot(TZRot)

        # -- Create Frame EE (Do the same for the end effector frame)
        T5EE = np.array([[1, 0, 0, L4 * cos(q1)],
                         [0, 1, 0, L4 * sin(q1)],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

        TEE = T05.dot(T5EE).dot(TZRot)  # translate and rotate !

        # --- Create the frames for the links which attach to the hoarm
        q3_a = pi - (- q3)  # adjusted q3 value

        # --- --- For Frame A

        T2A_rot = np.array([[cos(q3_a),  -sin(q3_a), 0, 0],  # Rotation about z axis by q2
                            [sin(q3_a),   cos(q3_a), 0, 0],
                            [0,         0,       1, 0],
                            [0,         0,       0, 1]])

        T2A_trans = np.array([[1, 0, 0, L2A],  # Translation along x axis by distance L2A
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        T2A = T2A_rot.dot(T2A_trans)

        T0A = T02.dot(T2A)  # Frame for co-ordinate A

        # --- --- For Frame B

        TAB_rot = np.array([[cos(-(q3_a)),  -sin(-(q3_a)), 0, 0],  # Rotation about z axis by -(180-q2)
                            [sin(-(q3_a)),   cos(-(q3_a)), 0, 0],
                            [0,            0,          1, 0],
                            [0,            0,          0, 1]])

        TAB_trans = np.array([[1, 0, 0, LAB],  # Translation along x axis by distance L2A
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        TAB = TAB_rot.dot(TAB_trans)

        T0B = T0A.dot(TAB)  # Frame for co-ordinate A

        # Defines the position of each joint.
        # Joint 1 (This acts as the base)
        Base = np.array([0, 0, 0])
        J2_Pos = np.array([T01[0, 3], T01[1, 3], T01[2, 3]])  # Joint 2
        J3_Pos = np.array([T02[0, 3], T02[1, 3], T02[2, 3]])  # Joint 3
        # Joint 4. it is assumed that the origin of joint 4 and 5 coincide.
        J4_Pos = np.array([T03[0, 3], T03[1, 3], T03[2, 3]])
        J5_Pos = np.array([T04[0, 3], T04[1, 3], T04[2, 3]])
        EE_Pos = np.array([TEE[0, 3], TEE[1, 3], TEE[2, 3]])
        # --- Horarm joints
        A_Pos = np.array([T0A[0, 3], T0A[1, 3], T0A[2, 3]])
        B_Pos = np.array([T0B[0, 3], T0B[1, 3], T0B[2, 3]])

        # Coordinates for each link.
        Link1 = list(zip(Base, J2_Pos))   # From base to Joint 2
        Link2 = list(zip(J2_Pos, J3_Pos))  # From Joint 2 to Joint 3
        Link3 = list(zip(J3_Pos, J4_Pos))  # From Joint 3 to Joint 4
        Link4 = list(zip(J4_Pos, J5_Pos))  # From Joint 4 to Joint 5
        Link5 = list(zip(J5_Pos, EE_Pos))  # From Joint 5 to the end effector
        # --- Horarm links
        LinkA = list(zip(J2_Pos, A_Pos))  # From Joint 2 to Joint A
        LinkB = list(zip(A_Pos, B_Pos))  # From Joint A to Joint B
        LinkC = list(zip(B_Pos, J4_Pos))  # From Joint B to Joint 4

        # create the figure
        fig = plt.figure()

        # add an axis
        ax = fig.add_subplot(111, projection='3d')

        # add labels to the plot
        ax.set(title="3-d simulation of EEZYbotARM", xlabel="x (mm)",
               ylabel="y (mm)", zlabel="z (mm)")

        # set axis equal
        # ax.set_aspect('equal')         # important!

        # data for lines
        linewidth = 3

        # plot the lines
        ax.plot(Link1[0], Link1[1], Link1[2],
                label="Link1", linewidth=linewidth)
        #ax.plot(Link2[0],Link2[1], Link2[2], label="Link2", linewidth=linewidth)
        ax.plot(Link3[0], Link3[1], Link3[2],
                label="Link2", linewidth=linewidth)
        ax.plot(Link4[0], Link4[1], Link4[2],
                label="Link3", linewidth=linewidth)
        ax.plot(Link5[0], Link5[1], Link5[2],
                label="Link4", linewidth=linewidth)

        # --- plot the lines for the horarm links
        ax.plot(LinkA[0], LinkA[1], LinkA[2],
                linewidth=linewidth, color='lightgrey')
        ax.plot(LinkB[0], LinkB[1], LinkB[2],
                linewidth=linewidth, color='lightgrey')
        ax.plot(LinkC[0], LinkC[1], LinkC[2],
                linewidth=linewidth, color='lightgrey')

        # add a legend
        ax.legend()

        # plot co-ordinate frames
        plotCoOrd(T00, ax, lineColor='blue')
        plotCoOrd(T02, ax, lineColor='orange')
        plotCoOrd(T03, ax, lineColor='green')
        plotCoOrd(T05_true, ax, lineColor='red')
        plotCoOrd(TEE, ax, lineColor='grey')

    #     plotCoOrd(T0A, ax, lineColor='lightgrey') # uncommenting will show servo 3 linkage co-ordinate frames
    #     plotCoOrd(T0B, ax, lineColor='lightgrey')

        # show the plot
        # important code such that all axes display with equal scaling in 3d!
        set_axes_equal(ax)
        plt.show()

        # define the position of the end effector
        x_EE = round(float(TEE[0, 3]), 3)
        y_EE = round(float(TEE[1, 3]), 3)
        z_EE = round(float(TEE[2, 3]), 3)

        # Debug step
        print("plot_EEZYbotARM function --> End effector position (mm) is x: {}, y:{}, z:{}".format(x_EE, y_EE, z_EE))

        return fig, ax

    def plotWithHull(self, **kwargs):
        """
        --Description-- [TEST FOR PLOTTING WITH HULL!]
        Given angles q1, q2, q3, plot EEZybot arm and co-ordinate frames for all joints using matplotlib
        Function computes the configuration of the manipulator and plots it using DH proximal convention.

        The function will raise an exception if any of the angles are outside defined limits

        The reference angle definitions are as follows:

        - EzzyBot base (q1) : 0 degree position is facing directly forwards
        - Main arm position (q2)
        - Horarm position(q3)

        --Required--
        Helper functions required (included in the EEzybot code)
        - plotCoOrd()
        - set_axes_radius()
        - set_axes_equal()

        --Optional **kwargs parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        fig, ax -> matplotlib figure and axes objects showing position of all actuator joints and links in world frame

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Convert angles to radians
        q1 = q1 * pi/180
        q2 = q2 * pi/180
        q3 = q3 * pi/180

        # DH parameters (Proximal Convention),
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4
        # Lengths for the links which attach to the hoarm
        L2A = self.L2A
        LAB = self.LAB
        LB3 = self.LB3

        # DH table
        DH = np.array([[0,  0,     L1, q1],
                       [0,  pi/2,  0,  q2],
                       [L2, 0,     0,  q3],
                       [L3, 0,     0,  0],
                       [0,  -pi/2, 0,  0]])

        # Find number of rows in DH table
        rows, cols = DH.shape

        # Pre-allocate Array to store Transformation matrix
        T = np.zeros((4, 4, rows), dtype=float)

        # Determine transformation matrix between each frame
        for i in range(rows):
            T[:, :, i] = [[cos(DH[i, 3]),              -sin(DH[i, 3]),               0,             DH[i, 0]],
                          [sin(DH[i, 3])*cos(DH[i, 1]),  cos(DH[i, 3]) *
                           cos(DH[i, 1]), -sin(DH[i, 1]), -sin(DH[i, 1])*DH[i, 2]],
                          [sin(DH[i, 3])*sin(DH[i, 1]),  cos(DH[i, 3]) *
                           sin(DH[i, 1]),  cos(DH[i, 1]),  cos(DH[i, 1])*DH[i, 2]],
                          [0,                          0,                          0,             1]]

        # Create the transformation frames with repect to the world frame (the base of the EEzybot arm)

        # --- Calculate Transformation matrix to each frame wrt the base. (matrix dot multiplication)
        T00 = np.identity(4)
        T01 = T[:, :, 0]
        T02 = T[:, :, 0].dot(T[:, :, 1])
        T03 = T[:, :, 0].dot(T[:, :, 1]).dot(T[:, :, 2])
        T04 = T[:, :, 0].dot(T[:, :, 1]).dot(T[:, :, 2]).dot(T[:, :, 3])

        # --- Create frame 5 (note this is just a rotation of frame T04)
        R5 = T04[0:3, 0:3]  # Find rotation matrix
        T45 = np.zeros((4, 4))
        T45[0:3, 0:3] = np.linalg.inv(R5)
        T45[3, 3] = 1  # Create transformation matrix from frame 4 to frame 5

        # Create the transformation matrix from the world frame to frame 5 (without z rotation)
        T05 = T04.dot(T45)

        # --- Construct a transformation matrix to make the Z rotation of T05 by magnitude q1
        TZRot = np.array([[cos(q1),  -sin(q1), 0, 0],
                          [sin(q1),   cos(q1), 0, 0],
                          [0,         0,       1, 0],
                          [0,         0,       0, 1]])

        # Create the transformation matrix from the world frame to frame 5 (with z rotation)
        T05_true = T05.dot(TZRot)

        # -- Create Frame EE (Do the same for the end effector frame)
        T5EE = np.array([[1, 0, 0, L4 * cos(q1)],
                         [0, 1, 0, L4 * sin(q1)],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

        TEE = T05.dot(T5EE).dot(TZRot)  # translate and rotate !

        # --- Create the frames for the links which attach to the hoarm
        q3_a = pi - (- q3)  # adjusted q3 value

        # --- --- For Frame A

        T2A_rot = np.array([[cos(q3_a),  -sin(q3_a), 0, 0],  # Rotation about z axis by q2
                            [sin(q3_a),   cos(q3_a), 0, 0],
                            [0,         0,       1, 0],
                            [0,         0,       0, 1]])

        T2A_trans = np.array([[1, 0, 0, L2A],  # Translation along x axis by distance L2A
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        T2A = T2A_rot.dot(T2A_trans)

        T0A = T02.dot(T2A)  # Frame for co-ordinate A

        # --- --- For Frame B

        TAB_rot = np.array([[cos(-(q3_a)),  -sin(-(q3_a)), 0, 0],  # Rotation about z axis by -(180-q2)
                            [sin(-(q3_a)),   cos(-(q3_a)), 0, 0],
                            [0,            0,          1, 0],
                            [0,            0,          0, 1]])

        TAB_trans = np.array([[1, 0, 0, LAB],  # Translation along x axis by distance L2A
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        TAB = TAB_rot.dot(TAB_trans)

        T0B = T0A.dot(TAB)  # Frame for co-ordinate A

        # Defines the position of each joint.
        # Joint 1 (This acts as the base)
        Base = np.array([0, 0, 0])
        J2_Pos = np.array([T01[0, 3], T01[1, 3], T01[2, 3]])  # Joint 2
        J3_Pos = np.array([T02[0, 3], T02[1, 3], T02[2, 3]])  # Joint 3
        # Joint 4. it is assumed that the origin of joint 4 and 5 coincide.
        J4_Pos = np.array([T03[0, 3], T03[1, 3], T03[2, 3]])
        J5_Pos = np.array([T04[0, 3], T04[1, 3], T04[2, 3]])
        EE_Pos = np.array([TEE[0, 3], TEE[1, 3], TEE[2, 3]])
        # --- Horarm joints
        A_Pos = np.array([T0A[0, 3], T0A[1, 3], T0A[2, 3]])
        B_Pos = np.array([T0B[0, 3], T0B[1, 3], T0B[2, 3]])

        # Coordinates for each link.
        Link1 = list(zip(Base, J2_Pos))   # From base to Joint 2
        Link2 = list(zip(J2_Pos, J3_Pos))  # From Joint 2 to Joint 3
        Link3 = list(zip(J3_Pos, J4_Pos))  # From Joint 3 to Joint 4
        Link4 = list(zip(J4_Pos, J5_Pos))  # From Joint 4 to Joint 5
        Link5 = list(zip(J5_Pos, EE_Pos))  # From Joint 5 to the end effector
        # --- Horarm links
        LinkA = list(zip(J2_Pos, A_Pos))  # From Joint 2 to Joint A
        LinkB = list(zip(A_Pos, B_Pos))  # From Joint A to Joint B
        LinkC = list(zip(B_Pos, J4_Pos))  # From Joint B to Joint 4

        # create the figure
        fig = plt.figure()

        # add an axis
        ax = fig.add_subplot(111, projection='3d')

        # add labels to the plot
        ax.set(title="3-d plot for EEZYbotARM Mk2 manipulator in XYZ world frame", xlabel="x (mm)",
               ylabel="y (mm)", zlabel="z (mm)")

        # set axis equal
        ax.set_aspect('equal')         # important!

        # Set limits for plot
#         ax.set_xlim([-350, 350])
#         ax.set_ylim([-350, 350])
#         ax.set_zlim([-350, 350])
        # data for lines
        linewidth = 3

        # plot the lines
        ax.plot(Link1[0], Link1[1], Link1[2],
                label="Link1", linewidth=linewidth)
        #ax.plot(Link2[0],Link2[1], Link2[2], label="Link2", linewidth=linewidth)
        ax.plot(Link3[0], Link3[1], Link3[2],
                label="Link2", linewidth=linewidth)
        ax.plot(Link4[0], Link4[1], Link4[2],
                label="Link3", linewidth=linewidth)
        ax.plot(Link5[0], Link5[1], Link5[2],
                label="Link4", linewidth=linewidth)

        # --- plot the lines for the horarm links
        ax.plot(LinkA[0], LinkA[1], LinkA[2],
                linewidth=linewidth, color='lightgrey')
        ax.plot(LinkB[0], LinkB[1], LinkB[2],
                linewidth=linewidth, color='lightgrey')
        ax.plot(LinkC[0], LinkC[1], LinkC[2],
                linewidth=linewidth, color='lightgrey')

        # add a legend
        ax.legend()

        # plot co-ordinate frames
        plotCoOrd(T00, ax, lineColor='blue')
        plotCoOrd(T02, ax, lineColor='orange')
        plotCoOrd(T03, ax, lineColor='green')
        plotCoOrd(T05_true, ax, lineColor='red')
        plotCoOrd(TEE, ax, lineColor='black')

    #     plotCoOrd(T0A, ax, lineColor='lightgrey') # uncommenting will show servo 3 linkage co-ordinate frames
    #     plotCoOrd(T0B, ax, lineColor='lightgrey')

        # ---Add the plot of the complex hull---

        # Define directory to load the convex hull
        self.directory = '/data'
        self.convexHullFilename = '/EEZYbotARM_workspace/workspace_convexHull.p'

        # Load the convex hull
        with open(self.directory+self.convexHullFilename, 'rb') as fp:
            simplical_facet_corners = pickle.load(fp)

        facetCol = [0.5, 0.8, 0.4, 0.3]
        lineCol = [0.8, 0.8, 0.8, 0.2]

        # Plot surface traingulation of the convex hull
        for simplical_facet_corner in simplical_facet_corners:
            vtx = simplical_facet_corner
            tri = a3.art3d.Poly3DCollection([vtx], linewidths=1, alpha=0.3)
            tri.set_color(facetCol)
            tri.set_edgecolor(lineCol)
            ax.add_collection3d(tri)

        # show the plot
        # important code such that all axes display with equal scaling in 3d!
        set_axes_equal(ax)
        # plt.show()

        # define the position of the end effector
        x_EE = round(float(TEE[0, 3]), 3)
        y_EE = round(float(TEE[1, 3]), 3)
        z_EE = round(float(TEE[2, 3]), 3)

        # Debug step
        print("plot_EEZYbotARM function --> End effector position (mm) is x: {}, y:{}, z:{}".format(x_EE, y_EE, z_EE))

        return fig, ax

    def plot_EEZYbotARM_workspace(self, stepSize=20, **kwargs):
        """
        --Description--
        Given angles q1, q2, q3

        --Required--
        Helper functions required (included in the EEzybot code)
        - plotCoOrd()
        - set_axes_radius()
        - set_axes_equal()

        --Optional **kwargs parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        fig -> matplotlib figure showing position of all actuator joints and links in world frame
        x_EE, y_EE, z_EE -> position of the end effector in the world frame (mm)

        """
        # DH parameters (Proximal Convention),
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        # Angle limits
        q1_min = self.q1_min * pi/180  # radians
        q1_max = self.q1_max * pi/180
        q1_numSteps = stepSize

        q2_min = self.q2_min * pi/180
        q2_max = self.q2_max * pi/180
        q2_numSteps = stepSize

        q3_numSteps = stepSize  # max, min not yet known due to dependance on q2

        # Step sizes
        q1_stepSize = (q1_max-q1_min)/q1_numSteps
        q2_stepSize = (q2_max-q2_min)/q2_numSteps

        # Size of pre-allocation for array
        arraySize = int((q1_numSteps+1) * (q2_numSteps+1) * (q3_numSteps+1))

        # Pre-allocation of arrays for results
        xEndEffectorPoints = np.zeros(arraySize, dtype=float)
        yEndEffectorPoints = np.zeros(arraySize, dtype=float)
        zEndEffectorPoints = np.zeros(arraySize, dtype=float)

        # Array index
        index = 0

        # set intial variable value
        q1 = q1_min

        # Loop through q1 angles
        for i in range(0, q1_numSteps+1):
            # for i in range(0, 1):

            print("Progress -> {} / {}".format(i, q1_numSteps))
        #     print("q1 = ", q1*180/pi)

            # set inital q2 value
            q2 = q2_min

            # increment index
            i += 1

            # Loop through q2 angles
            for j in range(0, q2_numSteps+1):

                #         print("q2 = ", q2 *180/pi)
                #         print("q2 stepsize= ", q2_stepSize*180/pi)

                # calculate q3 limits for this q2 angle
                temp = (q2*180/pi)
                q3_min, q3_max = self.q3CalcLimits(q2=temp)

                # convert to radians
                q3_min = q3_min * pi/180
                q3_max = q3_max * pi/180
                # set intial variable value
                q3 = q3_min
                # calculate q3 stepsize
                q3_stepSize = (q3_max-q3_min)/q3_numSteps

                # increment index
                j += 1

                # Loop through q3 angles
                for k in range(0, q3_numSteps+1):

                    #             print("q3 = ", q3*180/pi)

                    # Calculate end effector x, y, z position and store it
                    xEndEffectorPoints[index] = round(
                        (cos(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*cos(q1)), 1)
                    yEndEffectorPoints[index] = round(
                        (sin(q1) * (cos(q2+q3)*L3 + cos(q2)*L2))+(L4*sin(q1)), 1)
                    zEndEffectorPoints[index] = round(
                        (L1 + sin(q2)*L2 + sin(q2+q3)*L3), 1)

                    # increment loop index
                    k += 1
                    # increment array index
                    index += 1
                    # increase q3 value by stepsize
                    q3 = q3 + q3_stepSize

                # increase q2 value by stepsize
                q2 = q2 + q2_stepSize

            # increase q1 value by stepsize
            q1 = q1 + q1_stepSize

        # create the figure
        fig_workspace = plt.figure()

        # add an axis
        ax_workspace = fig_workspace.add_subplot(111, projection='3d')

        # add labels to the plot
        ax_workspace.set(title="3-d plot for EEZYbotARM Mk2 workspace in XYZ world frame", xlabel="x (mm)",
                         ylabel="y (mm)", zlabel="z (mm)")

        # plot the data
        ax_workspace.scatter(
            xEndEffectorPoints[0:index], yEndEffectorPoints[0:index], zEndEffectorPoints[0:index], c='red')

        # define the world frame
        T00 = np.identity(4)

        # plot co-ordinate frames
        plotCoOrd(T00, ax_workspace, lineColor='blue')

        # show the plot
        # important code such that all axes display with equal scaling in 3d!
        set_axes_equal(ax_workspace)
        # plt.show()

        # Code added in v3.1 to save the end effector points array
        # Append the sampled points so that they are in the same numpy array
        endEffectorPointsXYZ = xEndEffectorPoints[0:index].copy()
        endEffectorPointsXYZ = np.expand_dims(
            endEffectorPointsXYZ, axis=1)  # Equivalent to x[:,np.newaxis]
        endEffectorPointsXYZ = np.append(endEffectorPointsXYZ, np.expand_dims(
            yEndEffectorPoints[0:index], axis=1), axis=1)
        endEffectorPointsXYZ = np.append(endEffectorPointsXYZ, np.expand_dims(
            zEndEffectorPoints[0:index], axis=1), axis=1)

        # Save the array to a numpy object in the root directory
        self.directory = '/data'
        self.workspaceFilename = '/EEZYbotARM_workspace/workspace_endEffectorPointsXYZ.npy'
        pathlib.Path(self.directory).mkdir(parents=True, exist_ok=True)
        # .npy extension is added if not given
        np.save(self.directory+self.workspaceFilename, endEffectorPointsXYZ)
        print("Numpy array of workspace points saved to {} as {}".format(
            self.directory, self.workspaceFilename))

        return endEffectorPointsXYZ

    def createWorkspaceConvexHull(self):

        # Load the numpy array of sampled workspace end effector points
        print("Loading numpy array {}{}".format(
            self.directory, self.workspaceFilename))
        points = np.load(self.directory+self.workspaceFilename)

        print(points.shape)

        # Calculate the complex hull!
        hull = ConvexHull(points)

        # create the figure

#         fig_workspace = plt.figure()

#         #add an axis
#         ax = fig_workspace.add_subplot(111, projection='3d')
        rows, cols = hull.simplices.shape

        # Plot surface traingulation
        simplical_facet_corners = []

        for counter, simplex in enumerate(hull.simplices):
            simplical_facet_corners.append(
                [points[simplex[0], :], points[simplex[1], :], points[simplex[2], :]])

        # Define directory to save convex hull
        self.directory = '/data'
        self.convexHullFilename = '/EEZYbotARM_workspace/workspace_convexHull.p'

        with open(self.directory+self.convexHullFilename, 'wb') as fp:
            pickle.dump(simplical_facet_corners, fp)

        # Plotting preperation
        ax = a3.Axes3D(pl.figure())
        # facetCol = sp.rand(3) #[0.0, 1.0, 0.0]
        facetCol = [0.5, 0.8, 0.4]
        lineCol = [0.8, 0.8, 0.8, 0.5]

        # Plot surface traingulation
        for simplex in hull.simplices:
            vtx = [points[simplex[0], :],
                   points[simplex[1], :], points[simplex[2], :]]
            tri = a3.art3d.Poly3DCollection([vtx], linewidths=1, alpha=0.4)
            tri.set_color(facetCol)
            tri.set_edgecolor(lineCol)
            ax.add_collection3d(tri)

        # Set limits for plot
        ax.set_xlim([-350, 350])
        ax.set_ylim([-350, 350])
        ax.set_zlim([-350, 350])

        # plt.axis('off')
        plt.show()


# ------------------------------------------#
# Child class [inherits from EEZYbotARM() class]
# ------------------------------------------#

class EEZYbotARM_Mk2(EEZYbotARM):
    """
    --Description--
    This is a child class for the EEzybot Robotic arm MK2 (inherits from EEZYbotARM() class) 

    **Please note that you must always instantiate on a child class (either EEZYbotARMMk2() or EEZYbotARMMk1()) 
    rather than instansiating the parent 'EEZYbotARM' class**

    --Methods--
    Description of available methods in this class:    
        - q3CalcLimits --> Calculate the physical limits for joint 3 (because these depend on the angle of joint 2)
        - map_kinematicsToServoAngles --> Map angles defined in kinematics to physical servo angles on the robot
    """

    # Class attribute
    # DH parameters (Proximal Convention)
    L1 = 92  # mm
    L2 = 135
    L3 = 147
    L4 = 87

    # --- Lengths of links which attach to the hoarm
    L2A = 60
    LAB = 140
    LB3 = 60

    # Joint limits
    q1_min = -30  # degrees
    q1_max = 30
    q2_min = 39
    q2_max = 120
    # q3_min, q3_max are given by q3CalcLimits() function

    def q3CalcLimits(self, **kwargs):
        """
        Calculate q3 angle limits for the EzzyBot Arm given a value for the angle q2 in degrees
        These limits have been determined experimentally for the EEzybotMk2

        If no q2 value is given then the current value of q2 is used 

        --Optional kwargs Parameters--
        @q2 -> the value of the angle q2 (in degrees)

        --Returns--
        q3_min, q3_max -> the min and max limits for the angle q3 (in degrees)

        """
        # Use **kwarg if provided, otherwise use current q2 value
        q2 = kwargs.get('q2', self.q2)

        # calculate q3 min limits in degrees
        q3_min = (-0.6755 * q2) - 70.768
        q3_max = (-0.7165 * q2) - 13.144

        return q3_min, q3_max

    def map_kinematicsToServoAngles(self, **kwargs):
        """
        --Description--
        Take the three angles for the EzzyBot Arm defined in kinematics.
        Use these to calculate the required physical servo position with respect to the reference position.
        If three angles are not provided as **kwargs then the current values for the arm are used

        The reference positions for the three servos are as follows:

        EzzyBot base (q1) : 90 degree servo position is facing directly forwards
        Main arm (q2): 90 degree servo position is with main arm perpendicular (at 90 degrees to) base
        Horarm (q3): 90 degree servo poisition is with horarm servo link at 45 degrees to base

        The function will be updated to raise an error message when any of the returned angles are outside of the servo limits.

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        servoAngle_q1, servoAngle_q2, servoAngle_q3 -> values in degrees for output to the physical servos

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Check none of the angles are outside of joint limits! So that servos cannot get damaged
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)

        # Calculate for q1
        servoAngle_q1 = ((-2.0497)*q1) + 91.726  # from experimentation !
        servoAngle_q1 = round(servoAngle_q1, 2)

        # Calculate for q2
        servoAngle_q2 = 180 - q2  # approximate adjusted q2 value
        servoAngle_q2 = round(servoAngle_q2, 2)

        # Calculate for q3
        q3_a = 180 - (- q3)  # approximate adjusted q3 value
        servoAngle_q3 = q2 - 45 + q3_a
        servoAngle_q3 = round(servoAngle_q3, 2)

        return servoAngle_q1, servoAngle_q2, servoAngle_q3


class EEZYbotARM_Mk1(EEZYbotARM):
    """
    --Description--
    This is a child class for the EEzybot Robotic arm MK1 (inherits from EEZYbotARM() class) 

    **Please note that you must always instantiate on a child class (either EEZYbotARMMk2() or EEZYbotARMMk1()) 
    rather than instansiating the parent 'EEZYbotARM' class**

    --Methods--
    Description of available methods in this class:    
        - q3CalcLimits --> Calculate the physical limits for joint 3 (because these depend on the angle of joint 2)
        - map_kinematicsToServoAngles --> Map angles defined in kinematics to physical servo angles on the robot
    """

    # Class attribute
    # DH parameters (Proximal Convention)
    L1 = 61  # mm
    L2 = 80
    L3 = 80
    L4 = 57

    # --- Lengths of links which attach to the hoarm
    L2A = 35
    LAB = 80
    LB3 = 35

    # Joint limits
    q1_min = -30  # degrees
    q1_max = 30
    q2_min = 39
    q2_max = 120
    # q3_min, q3_max are given by q3CalcLimits() function

    def q3CalcLimits(self, **kwargs):
        """
        Calculate q3 angle limits for the EzzyBot Arm given a value for the angle q2 in degrees
        These limits have been determined experimentally for the EEzybotMk1

        If no q2 value is given then the current value of q2 is used 

        --Optional kwargs Parameters--
        @q2 -> the value of the angle q2 (in degrees)

        --Returns--
        q3_min, q3_max -> the min and max limits for the angle q3 (in degrees)

        """
        # Use **kwarg if provided, otherwise use current q2 value
        q2 = kwargs.get('q2', self.q2)

        # calculate q3 min limits in degrees
        q3_min = (-0.6755 * q2) - 70.768
        q3_max = (-0.7165 * q2) - 13.144

        return q3_min, q3_max

    def map_kinematicsToServoAngles(self, **kwargs):
        """
        --Description--
        Take the three angles for the EzzyBot Arm defined in kinematics.
        Use these to calculate the required physical servo position with respect to the reference position.
        If three angles are not provided as **kwargs then the current values for the arm are used

        The reference positions for the three servos are as follows:

        EzzyBot base (q1) : 90 degree servo position is facing directly forwards
        Main arm (q2): 90 degree servo position is with main arm perpendicular (at 90 degrees to) base
        Horarm (q3): 90 degree servo poisition is with horarm servo link at 45 degrees to base

        The function will be updated to raise an error message when any of the returned angles are outside of the servo limits.

        --Optional **kwargs Parameters--
        @q1 -> the value of the angle q1 (in degrees) as used in the kinematics model
        @q2 -> the value of the angle q2 (in degrees) as used in the kinematics model
        @q3 -> the value of the angle q3 (in degrees) as used in the kinematics model

        --Returns--
        servoAngle_q1, servoAngle_q2, servoAngle_q3 -> values in degrees for output to the physical servos

        """

        # Use **kwargs if provided, otherwise use current values
        q1 = kwargs.get('q1', self.q1)
        q2 = kwargs.get('q2', self.q2)
        q3 = kwargs.get('q3', self.q3)

        # Check none of the angles are outside of joint limits! So that servos cannot get damaged
        self.checkErrorJointLimits(q1=q1, q2=q2, q3=q3)

        # Calculate for q1
        servoAngle_q1 = ((-2.0497)*q1) + 91.726  # from experimentation !
        servoAngle_q1 = round(servoAngle_q1, 2)

        # Calculate for q2
        servoAngle_q2 = 180 - q2  # approximate adjusted q2 value
        servoAngle_q2 = round(servoAngle_q2, 2)

        # Calculate for q3
        q3_a = 180 - (- q3)  # approximate adjusted q3 value
        servoAngle_q3 = q2 - 45 + q3_a
        servoAngle_q3 = round(servoAngle_q3, 2)

        return servoAngle_q1, servoAngle_q2, servoAngle_q3


# # ------------------------------------------#
# # Function for plotting robot arm
# # ------------------------------------------#

# class plot_EEZYbotARM(robot_arm):

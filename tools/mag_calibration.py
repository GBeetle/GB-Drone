import numpy as np
from scipy import linalg
from matplotlib import pyplot as plt
from scipy.interpolate import griddata
from mpl_toolkits.mplot3d import Axes3D

class Magnetometer(object):

    '''
     references :
        -  https://teslabs.com/articles/magnetometer-calibration/

    '''
    MField = 1

    def __init__(self, F=MField):
        # initialize values
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)

    def run(self):
        limits = 1.5
        # input data in Gauss: x,y,z
        data = np.loadtxt("uncalibration_data.csv",delimiter=',')
        #print("shape of data:",data.shape)
        #print("datatype of data:",data.dtype)
        #print("First 5 rows raw:\n", data[:5])

        # ellipsoid fit
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        # calibration parameters
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

        #print("M:\n", M, "\nn:\n", n, "\nd:\n", d)
        #print("M_1:\n",M_1, "\nb:\n", self.b, "\nA_1:\n", self.A_1)

        print("Soft iron transformation matrix:\n",self.A_1)
        print("Hard iron bias:\n", self.b)

        plt.rcParams["figure.autolayout"] = True
        #fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')
        #ax.scatter(data[:,0], data[:,1], data[:,2], marker='o', color='r')
        #plt.show()

        result = []
        for row in data:
            u = np.array(row).reshape(3, 1)
            m = np.dot(self.A_1, u - self.b)
            result = np.append(result, m)

        result = result.reshape(-1, 3)
        saved_result = np.array(result)
        np.savetxt('calibration_data.csv', saved_result, fmt='%f', delimiter=',')

        fig = plt.figure()
        ax1 = fig.add_subplot(111, projection='3d')
        #scatter plot
        ax1.scatter(data[:,0], data[:,1], data[:,2], marker='o', color='r', s=1, label='Origin Data')
        ax1.scatter(result[:, 0], result[:, 1], result[:, 2], marker='o', color='b', s=1, label='Calibrated Data')
        # Set the axis limits
        ax1.set_xlim3d(-limits, limits)
        ax1.set_ylim3d(-limits, limits)
        ax1.set_zlim3d(-limits, limits)

        """
        grid_x, grid_y = np.mgrid[
            np.min(data[:, 0]):np.max(data[:, 0]):100j,
            np.min(data[:, 1]):np.max(data[:, 1]):100j
        ]
        grid_z = griddata(data[:, 0:2], data[:, 2], (grid_x, grid_y), method='cubic')
        ax1.plot_surface(grid_x, grid_y, grid_z, color='r', alpha=0.1)
        ax1.plot_wireframe(grid_x, grid_y, grid_z, color='k', linewidth=0.2)
        """

        """
        # Plot the calibrated data
        ax2 = fig.add_subplot(122, projection='3d')
        ax2.scatter(result[:, 0], result[:, 1], result[:, 2], marker='o', color='b', s=1)
        ax2.set_title('Calibrated Data')

        # Set the axis limits
        ax2.set_xlim3d(-limits, limits)
        ax2.set_ylim3d(-limits, limits)
        ax2.set_zlim3d(-limits, limits)
        """

        """
        grid_x2, grid_y2 = np.mgrid[
            np.min(result[:, 0]):np.max(result[:, 0]):100j,
            np.min(result[:, 1]):np.max(result[:, 1]):100j
        ]
        grid_z2 = griddata(result[:, 0:2], result[:, 2], (grid_x2, grid_y2), method='cubic')
        ax2.plot_surface(grid_x2, grid_y2, grid_z2, color='r', alpha=0.1)
        ax2.plot_wireframe(grid_x2, grid_y2, grid_z2, color='k', linewidth=0.2)
        """

        plt.show()

        print("First 5 rows calibrated:\n", result[:5])
        np.savetxt('out.txt', result, fmt='%f', delimiter=' ,')

        """
        print("*************************" )
        print("code to paste : " )
        print("*************************" )
        print("float hard_iron_bias_x = ", self.b[0], ";")
        print("float hard_iron_bias_y = ", self.b[1], ";")
        print("float hard_iron_bias_z = ", self.b[2], ";")
        print("\n")
        print("double soft_iron_bias_xx = ", self.A_1[0,0], ";")
        print("double soft_iron_bias_xy = ", self.A_1[1,0], ";")
        print("double soft_iron_bias_xz = ", self.A_1[2,0], ";")
        print("\n")
        print("double soft_iron_bias_yx = ", self.A_1[0,1], ";")
        print("double soft_iron_bias_yy = ", self.A_1[1,1], ";")
        print("double soft_iron_bias_yz = ", self.A_1[2,1], ";")
        print("\n")
        print("double soft_iron_bias_zx = ", self.A_1[0,2], ";")
        print("double soft_iron_bias_zy = ", self.A_1[1,2], ";")
        print("double soft_iron_bias_zz = ", self.A_1[2,2], ";")
        print("\n")
        """

    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
        M = np.array([[v_1[0], v_1[5], v_1[4]],
                      [v_1[5], v_1[1], v_1[3]],
                      [v_1[4], v_1[3], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d

if __name__=='__main__':
        Magnetometer().run()
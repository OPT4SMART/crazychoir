from numpy.polynomial import Polynomial as poly
import matplotlib.pyplot as plt
import numpy as np
import csv
import traceback
import math
from .settings import SettingsParser

class DrawSpline:

    def __init__(self):

        self.x, self.y = None, None

    @staticmethod
    def plot_cubic_spline(csv_file_name, figure, ax, time_scale, interpolation_scale):

        x, y, t = DrawSpline.get_cords(csv_file_name)

        t = [value * time_scale for value in t]

        try:
            t_full = np.linspace(0, t[-1], 500)
            x, y, t = DrawSpline.__distance_interpolation__(x, y, t, interpolation_scale)

            x = list(x)
            y = list(y)
            t = list(t)
            t_full = np.linspace(0, t[-1], 500)


            t_extra = [
                    0.5*(t[0]+t[1]),
                    0.5*(t[-2]+t[-1])
                ] 
            
            t.insert(1,t_extra[0])
            t.insert(-1,t_extra[1])

            n_points = len(t)
            T = [t[i+1]-t[i] for i in range(n_points-1)]

            velocity_borders = [0,0]
            acceleration_borders = [0,0]
            coeff = []
            coeff.append(DrawSpline.__compute_spline_coefficients__(T,x,velocity_borders, acceleration_borders))
            coeff.append(DrawSpline.__compute_spline_coefficients__(T,y,velocity_borders, acceleration_borders))

            position, velocity, acceleration = DrawSpline.__spline_evaluation__(coeff, t_full, t)

            tmp = csv_file_name.split('/')
            file_name = tmp[len(tmp) - 1]
            message = f'Cubic - Spline of {file_name}'

            figure.suptitle(message)

            x1, = ax[0].plot(t, x, 'o', label='data')
            x2, = ax[0].plot(t_full, position[0], label="s(t)")
            x3, = ax[0].plot(t_full, velocity[0], label="v(t)")
            x4, = ax[0].plot(t_full, acceleration[0], label="a(t)")
            ax[0].legend(loc='lower left', ncol=2)
            ax[0].set_ylabel("x")
            ax[0].set_xlabel("t (seconds)")

            y1, = ax[1].plot(t, y, 'o', label='data')
            y2, = ax[1].plot(t_full, position[1], label="s(t)")
            y3, = ax[1].plot(t_full, velocity[1], label="v(t)")
            y4, = ax[1].plot(t_full, acceleration[1], label="a(t)")
            ax[1].legend(loc='lower left', ncol=2)
            ax[1].set_ylabel("y")
            ax[1].set_xlabel("t (seconds)")

            return [x1, x2, x3, x4], [y1, y2, y3, y4]

        except:
            traceback.print_exc()

    @staticmethod
    def get_cords(csv_file_name):

        with open(csv_file_name, newline='', encoding='utf-8') as csv_file:
            csv_reader = csv.reader(csv_file)
            next(csv_reader)
            x = []
            y = []
            t = []

            for row in csv_reader:
                x.append(round(float(row[0]), 3))
                y.append(round(float(row[1]), 3))
                t.append(round(float(row[2]), 3))

            return x, y, t

    @staticmethod
    def __compute_spline_coefficients__(T,q,a,v):
        """
        See Chapter §4.4.4 of "Trajectory planning for automatic machines and robots", L.Biagiotti, C. Melchiorri 
        """
        # Number of point extended to allow initial and final velociti and acceleration constraints
        N = len(q) + 1

        # Let's define the linear system Aw=c
        # Compute A
        A = np.zeros((N-1,N-1))
        for i in range(1,N-2):
            A[i,i-1] = T[i]
            A[i,i] = 2*(T[i] + T[i+1])
            A[i,i+1] = T[i+1]
        A[0,0] = 2*T[1] + T[0]*(3+T[0]/T[1])
        A[0,1] = T[1]
        A[1,0] -= T[0]*T[0]/T[1]
        A[-2,-1] -=  T[-1]*T[-1]/T[-2]
        A[-1,-2] = T[-2]
        A[-1,-1] = 2*T[-2]+T[-1]*(3+T[-1]/T[-2])

        # Compute c
        c = np.zeros(N-1)
        for i in range(1,N-2):
            c[i] = (q[i+1]-q[i])/T[i+1] -(q[i]-q[i-1])/T[i]
        c[0] = (q[1]-q[0])/T[1] - v[0]*(1+T[0]/T[1]) - a[0]*T[0]*(0.5+T[0]/T[1]/3)
        c[1] = (q[2]-q[1])/T[2] -(q[1]-q[0])/T[0] + v[0]*T[0]/T[1] + a[0]*T[0]*T[0]/T[1]/3
        c[-2] = (q[-1]-q[-2])/T[-2] -(q[-2]-q[-3])/T[-3] - v[1]*T[N-1]/T[N-2] + a[1]*T[N-1]*T[N-1]/T[N-2]/3
        c[-1] = (q[-2]-q[-1])/T[-2] + v[1]*(1+T[N-1]/T[N-2]) - a[1]*(0.5+T[N-1]/T[N-2]/3)*T[N-1]
        c *= 6

        # Solve the linear system finding the accellerations in the intermediate points
        w = (np.linalg.inv(A)@c).tolist()

        # Append initial and final accelerations
        w.insert(0,a[0])
        w.append(a[1])

        # Compute two extra point
        q_extra = [
                    q[0]+T[0]*v[0]+T[0]*T[0]*a[0]/3+T[0]*T[0]/6*w[1],
                    q[-1]-T[-1]*v[1]+T[-1]*T[-1]/3*a[1]+T[-1]*T[-1]/6*w[-2]
                ] 
        q.insert(1,q_extra[0])
        q.insert(N-1,q_extra[1])

        # Compute the spline coefficient
        a = np.zeros((N,4))
        for k in range(N):
            a[k,0] = q[k]
            a[k,1] = (q[k+1]-q[k])/T[k]-T[k]/6*(w[k+1]+2*w[k])
            a[k,2] = w[k]/2
            a[k,3] = (w[k+1]-w[k])/6/T[k]
            
        return a

    @staticmethod
    def __spline_evaluation__(coeff, time_horizon, time_instants):

        n_points = len(time_instants)

        coeff = np.array(coeff)
        position = [[],[]]
        velocity = [[],[]]
        acceleration = [[],[]]

        for time in time_horizon:
            for q in range(n_points-1):
                if time>= time_instants[q] and time <= time_instants[q+1]:
                    position[0].append(poly(coeff[0,q,:])(time-time_instants[q]))
                    position[1].append(poly(coeff[1,q,:])(time-time_instants[q]))
                    velocity[0].append(poly(coeff[0,q,1:].T*np.arange(1,4))(time-time_instants[q]))
                    velocity[1].append(poly(coeff[1,q,1:].T*np.arange(1,4))(time-time_instants[q]))
                    acceleration[0].append(poly(coeff[0,q,2:].T*np.arange(2,4)*np.arange(1,3))(time-time_instants[q]))
                    acceleration[1].append(poly(coeff[1,q,2:].T*np.arange(2,4)*np.arange(1,3))(time-time_instants[q]))
                    break
        
        return position, velocity, acceleration

    @staticmethod
    def __time_interpolation__(x, y, t, line_count, num_interpolation):

        tmp_x, tmp_y, tmp_t = [], [], []

        xs = np.arange(0, line_count - 1, int((line_count - 1) / num_interpolation))

        for i in xs:
            tmp_x.append(x[i])
            tmp_y.append(y[i])
            tmp_t.append(t[i])

        return np.array(tmp_x), np.array(tmp_y), np.array(tmp_t)

    @staticmethod
    def __distance_interpolation__(x, y, t, interval):

        tmp_x, tmp_y, tmp_t = [], [], []
        tmp_x.append(x[0])
        tmp_y.append(y[0])
        tmp_t.append(t[0])
        for i in range(len(t) - 1):
            # if math.dist([tmp_x[-1], tmp_y[-1]], [x[i + 1], y[i + 1]]) + tmp_dist > interval:
            if math.dist([tmp_x[-1], tmp_y[-1]], [x[i + 1], y[i + 1]]) > interval:
                tmp_x.append(x[i])
                tmp_y.append(y[i])
                tmp_t.append(t[i])
                # tmp_dist = 0
            # else:
            #     tmp_dist += math.dist([x[i], y[i]], [x[i + 1], y[i + 1]])

        return np.array(tmp_x), np.array(tmp_y), np.array(tmp_t)

    @staticmethod
    def print_poly_to_file(csv_file_name, output_file_name, interpolation_distance):
        raise NotImplementedError

        x, y, t = DrawSpline.get_cords(csv_file_name)
        x, y, t = DrawSpline.__distance_interpolation__(x, y, t, interpolation_distance)

        spline_x = CubicSpline(t, x)
        spline_y = CubicSpline(t, y)

        try:
            csv_file = open(output_file_name, 'w')

            csv_file.write('t_start, t_stop, ax^3, bx^2, cx, d, ay^3, by^2, cy, d\n')

            for i in range(num_interpolation):

                t_start = spline_x.x[i]
                t_stop = spline_x.x[i+1]
                x_poly = spline_x.c[:, i]
                y_poly = spline_y.c[:, i]

                csv_file.write(f'{t_start}, {t_stop}, {x_poly[0]}, {x_poly[1]}, {x_poly[2]}, {x_poly[3]}, {y_poly[0]}, {y_poly[1]}, {y_poly[2]}, {y_poly[3]}\n')

            csv_file.flush()
            csv_file.close()

        except:
            traceback.print_exc()

    def spline_check(csv_file_name, time_scale, interpolation_scale):

        x_raw, y_raw, t_raw = DrawSpline.get_cords(csv_file_name)

        t_raw = [value * time_scale for value in t_raw]

        try:
            
            t_full = np.linspace(0, t_raw[-1], 500)
            x, y, t = DrawSpline.__distance_interpolation__(x_raw, y_raw, t_raw, interpolation_scale)

            x = list(x)
            y = list(y)
            t = list(t)
            t_full = np.linspace(0, t[-1], 500)


            t_extra = [
                    0.5*(t[0]+t[1]),
                    0.5*(t[-2]+t[-1])
                ] 
            
            t.insert(1,t_extra[0])
            t.insert(-1,t_extra[1])

            n_points = len(t)
            T = [t[i+1]-t[i] for i in range(n_points-1)]

            velocity_borders = [0,0]
            acceleration_borders = [0,0]
            coeff = []
            coeff.append(DrawSpline.__compute_spline_coefficients__(T,x,velocity_borders, acceleration_borders))
            coeff.append(DrawSpline.__compute_spline_coefficients__(T,y,velocity_borders, acceleration_borders))

            position, _,_ = DrawSpline.__spline_evaluation__(coeff, t_full, t)


            plt.plot(x_raw, y_raw, color='r', label='raw data')
            plt.plot(position[0], position[1], color='g', label='interpolated')
            
            # Naming the x-axis, y-axis and the whole graph
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("Comparison between interpolated and raw data")
            
            # Adding legend, which helps us recognize the curve according to it's color
            plt.legend()
            
            # To load the display window
            plt.show()

        except:
            traceback.print_exc()


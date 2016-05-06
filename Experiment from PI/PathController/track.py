import math


class InvalidTrackParametersException(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class Track:
    def __init__(self):
        self.cubic = False
        self.x_planning = []
        self.y_planning = []
        self.z_planning = []
        self.t_planning = []
        self.constant_k = 0
        self.constant_t = 0

        self.xd_vector = []
        self.yd_vector = []
        self.zd_vector = []
        self.xd_dot_vector = []
        self.yd_dot_vector = []
        self.zd_dot_vector = []

        self.n_points = 0

    def generate(self, x_planning=None, y_planning=None, sample_time=None,
                 z_planning=None, t_planning=None, constant_k=None,
                 constant_t=None, cubic=False):
        if not (len(x_planning) == len(y_planning)):
            raise InvalidTrackParametersException('X and Y Vector for planning must have same length.')
        if len(x_planning) < 2:
            raise InvalidTrackParametersException('Number of points must be at least 2.')

        self.cubic = cubic
        self.x_planning = x_planning
        self.y_planning = y_planning
        self.z_planning = z_planning
        self.t_planning = t_planning
        self.constant_k = constant_k
        self.constant_t = constant_t

        if cubic:
            self.generate_cubic(x_planning, y_planning, sample_time, z_planning, t_planning, constant_k, constant_t,
                                cubic)
        else:
            self.generate_lineal(x_planning, y_planning, sample_time, z_planning, t_planning, constant_k, constant_t,
                                 cubic)

    def generate_cubic(self, x_planning=None, y_planning=None, sample_time=None,
                       z_planning=None, t_planning=None, constant_k=None,
                       constant_t=None, cubic=False):
        if not constant_k > 0 and constant_t > 0:
            raise InvalidTrackParametersException('Both constants T and K must be positive.')

        dn = constant_t / float(sample_time)

        n = int(dn)

        intervals = len(x_planning) - 1

        if (n * intervals) > 2000:
            raise InvalidTrackParametersException('Too much points.')

        self.xd_vector = range(n * intervals)
        self.yd_vector = range(n * intervals)
        self.zd_vector = range(n * intervals)

        self.xd_dot_vector = range(n * intervals)
        self.yd_dot_vector = range(n * intervals)
        self.zd_dot_vector = range(n * intervals)

        self.n_points = n * intervals

        for j in range(0, intervals):
            x_i = x_planning[j]
            y_i = y_planning[j]
            theta_i = z_planning[j]

            x_f = x_planning[j + 1]
            y_f = y_planning[j + 1]
            theta_f = z_planning[j + 1]

            alpha_x = constant_k * math.cos(theta_f) - 3 * x_f
            alpha_y = constant_k * math.sin(theta_f) - 3 * y_f

            beta_x = constant_k * math.cos(theta_i) + 3 * x_i
            beta_y = constant_k * math.sin(theta_i) + 3 * y_i

            for i in range(0, n):
                s = i / float(n)
                self.xd_vector[n * j + i] = - (s - 1) * (s - 1) * (s - 1) * x_i + s * s * s * x_f + alpha_x * (
                    s * s) * (s - 1) + beta_x * s * ((s - 1) * (s - 1))
                self.yd_vector[n * j + i] = - (s - 1) * (s - 1) * (s - 1) * y_i + s * s * s * y_f + alpha_y * (
                    s * s) * (s - 1) + beta_y * s * ((s - 1) * (s - 1))

                self.xd_dot_vector[n * j + i] = - 3 * (s - 1) * (s - 1) * x_i + 3 * s * s * x_f + alpha_x * (
                    3 * s * s - 2 * s) + beta_x * (3 * s * s - 4 * s + 1)
                self.yd_dot_vector[n * j + i] = - 3 * (s - 1) * (s - 1) * y_i + 3 * s * s * y_f + alpha_y * (
                    3 * s * s - 2 * s) + beta_y * (3 * s * s - 4 * s + 1)

                xd_dot_dot = - 6 * (s - 1) * x_i + 6 * s * x_f + alpha_x * (6 * s - 2) + beta_x * (6 * s - 4)
                yd_dot_dot = - 6 * (s - 1) * y_i + 6 * s * y_f + alpha_y * (6 * s - 2) + beta_y * (6 * s - 4)

                self.zd_dot_vector[n * j + i] = \
                    (yd_dot_dot * self.xd_dot_vector[n * j + i] - xd_dot_dot * self.yd_dot_vector[n * j + i]) \
                    / (self.xd_dot_vector[n * j + i] * self.xd_dot_vector[n * j + i]
                       + self.yd_dot_vector[n * j + i] * self.yd_dot_vector[n * j + i])

                self.xd_dot_vector[n * j + i] = self.xd_dot_vector[n * j + i] / constant_t
                self.yd_dot_vector[n * j + i] = self.yd_dot_vector[n * j + i] / constant_t

                self.zd_dot_vector[n * j + i] = self.zd_dot_vector[n * j + i] / constant_t

            self.zd_vector[n * j] = theta_i

            for i in range(0, n - 1):
                self.zd_vector[n * j + i + 1] = self.zd_vector[n * j + i] + self.zd_dot_vector[
                    n * j + i] * sample_time

    def generate_lineal(self, x_planning=None, y_planning=None, sample_time=None,
                        z_planning=None, t_planning=None, constant_k=None,
                        constant_t=None, cubic=False):
        if not len(x_planning) == len(t_planning):
            raise InvalidTrackParametersException('T vector must have the same length as points.')

        max_count = 0
        theta_p_i = 0

        for i in range(0, len(x_planning) - 1):

            if t_planning[i] < 0:
                raise InvalidTrackParametersException('All values of T vector must be positive.')

            i_points = int(t_planning[i + 1] / sample_time)
            max_count += int(i_points)

            if max_count > 2000:
                raise InvalidTrackParametersException('Too much points.')

        self.xd_vector = range(max_count)
        self.yd_vector = range(max_count)
        self.zd_vector = range(max_count)

        self.xd_dot_vector = range(max_count)
        self.yd_dot_vector = range(max_count)
        self.zd_dot_vector = range(max_count)

        self.n_points = max_count

        max_count = 0

        for i in range(0, len(x_planning) - 1):

            i_points = int(t_planning[i + 1] / sample_time)
            base_interval_index = max_count
            max_count += i_points

            self.xd_vector[base_interval_index] = x_planning[i]
            self.yd_vector[base_interval_index] = y_planning[i]

            delta_x = (x_planning[i + 1] - x_planning[i]) / float(i_points)
            delta_y = (y_planning[i + 1] - y_planning[i]) / float(i_points)

            xd_dot = delta_x / sample_time
            yd_dot = delta_y / sample_time
            theta_i = math.atan2(yd_dot, xd_dot)

            diff1 = abs(theta_i - theta_p_i)
            diff2 = abs(theta_i + 2 * math.pi - theta_p_i)
            diff3 = abs(theta_i - 2 * math.pi - theta_p_i)

            if diff2 < diff1:

                if diff2 < diff3:
                    theta_i += 2 * math.pi
                else:
                    theta_i -= 2 * math.pi
            else:
                if diff3 < diff1:
                    theta_i += 2 * math.pi

            self.xd_dot_vector[base_interval_index] = xd_dot
            self.yd_dot_vector[base_interval_index] = yd_dot
            self.zd_vector[base_interval_index] = theta_i
            self.zd_dot_vector[base_interval_index] = (theta_i - theta_p_i) / float(sample_time)

            theta_p_i = theta_i

            for j in range(1, i_points):
                self.xd_vector[base_interval_index + j] = self.xd_vector[base_interval_index + j - 1] + delta_x
                self.yd_vector[base_interval_index + j] = self.yd_vector[base_interval_index + j - 1] + delta_y
                self.zd_vector[base_interval_index + j] = theta_i
                self.xd_dot_vector[base_interval_index + j] = xd_dot
                self.yd_dot_vector[base_interval_index + j] = yd_dot
                self.zd_dot_vector[base_interval_index + j] = 0.0

    def generate_backward(self, x_planning=None, y_planning=None, sample_time=None,
                          z_planning=None, t_planning=None, constant_k=None,
                          constant_t=None, cubic=False):
        if len(x_planning) != 2 or len(y_planning) != 2:
            raise InvalidTrackParametersException('Only two values in vector are needed.')

        i_points = int(t_planning[1] / sample_time)

        if i_points > 2000:
            raise InvalidTrackParametersException('Too much points.')

        self.xd_vector = range(i_points)
        self.yd_vector = range(i_points)
        self.zd_vector = range(i_points)

        self.xd_dot_vector = range(i_points)
        self.yd_dot_vector = range(i_points)
        self.zd_dot_vector = range(i_points)

        self.n_points = i_points

        delta_x = x_planning[1] / float(i_points)
        delta_y = y_planning[1] / float(i_points)

        xd_dot = delta_x / sample_time
        yd_dot = delta_y / sample_time

        self.n_points = i_points

        for j in range(i_points):
            self.xd_vector[j] = j*delta_x
            self.yd_vector[j] = j*delta_y
            self.zd_vector[j] = 0.0
            self.xd_dot_vector[j] = xd_dot
            self.yd_dot_vector[j] = yd_dot
            self.zd_dot_vector[j] = 0.0

    def generate_rotation(self, x_planning=None, y_planning=None, sample_time=None,
                          z_planning=None, t_planning=None, constant_k=None,
                          constant_t=None, cubic=False):
        if len(z_planning) != 2:
            raise InvalidTrackParametersException('Only two values in vector are needed.')

        i_points = int(t_planning[1] / sample_time)

        if i_points > 2000:
            raise InvalidTrackParametersException('Too much points.')

        self.xd_vector = range(i_points)
        self.yd_vector = range(i_points)
        self.zd_vector = range(i_points)

        self.xd_dot_vector = range(i_points)
        self.yd_dot_vector = range(i_points)
        self.zd_dot_vector = range(i_points)

        self.n_points = i_points

        delta_z = z_planning[1] / float(i_points)
        zd_dot = delta_z / sample_time

        for j in range(i_points):
            self.xd_vector[j] = 0.0
            self.yd_vector[j] = 0.0
            self.zd_vector[j] = j * delta_z
            self.xd_dot_vector[j] = 0.0
            self.yd_dot_vector[j] = 0.0
            self.zd_dot_vector[j] = zd_dot


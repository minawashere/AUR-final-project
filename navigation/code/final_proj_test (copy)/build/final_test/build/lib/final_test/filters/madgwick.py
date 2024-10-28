import math

class MadgwickAHRS:
    def __init__(self, sampleperiod=1/256, beta=0.1):
        self.q = [1.0, 0.0, 0.0, 0.0]  # Quaternion of sensor frame relative to auxiliary frame
        self.beta = beta  # Madgwick gain
        self.sampleperiod = sampleperiod

    def update_imu(self, gx, gy, gz, ax, ay, az):
        # Convert gyroscope degrees/sec to radians/sec
        gx = gx * (math.pi / 180.0)
        gy = gy * (math.pi / 180.0)
        gz = gz * (math.pi / 180.0)

        # Normalize accelerometer
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 0:
            ax /= norm
            ay /= norm
            az /= norm

        # Quaternion update algorithm (Madgwick's method)
        q1, q2, q3, q4 = self.q
        f1 = 2 * (q2 * q4 - q1 * q3) - ax
        f2 = 2 * (q1 * q2 + q3 * q4) - ay
        f3 = 2 * (0.5 - q2 * q2 - q3 * q3) - az

        # Compute the Jacobian
        j11 = 2 * q3
        j12 = 2 * q4
        j13 = 2 * q1
        j14 = -2 * q2
        j21 = 2 * q2
        j22 = -2 * q1
        j23 = 2 * q4
        j24 = -2 * q3
        j31 = 0
        j32 = 2 * q3
        j33 = 2 * q2
        j34 = 2 * q1

        # Calculate the gradient descent step
        step = [j11 * f1 + j12 * f2 + j13 * f3,
                j21 * f1 + j22 * f2 + j23 * f3,
                j31 * f1 + j32 * f2 + j33 * f3]
        norm = math.sqrt(step[0] ** 2 + step[1] ** 2 + step[2] ** 2)

        if norm > 0:
            step = [s / norm for s in step]

        # Apply feedback step
        qDot1 = 0.5 * (q2 * gx - q3 * gy - q4 * gz) - self.beta * step[0]
        qDot2 = 0.5 * (q3 * gx + q2 * gy + q4 * gz) - self.beta * step[1]
        qDot3 = 0.5 * (q4 * gx - q2 * gy + q3 * gz) - self.beta * step[2]
        qDot4 = 0.5 * (q2 * gz + q3 * gy - q4 * gx)

        # Integrate to yield quaternion
        self.q[0] += qDot1 * self.sampleperiod
        self.q[1] += qDot2 * self.sampleperiod
        self.q[2] += qDot3 * self.sampleperiod
        self.q[3] += qDot4 * self.sampleperiod

        # Normalize the quaternion
        norm = math.sqrt(self.q[0] ** 2 + self.q[1] ** 2 + self.q[2] ** 2 + self.q[3] ** 2)
        if norm > 0:
            self.q = [q / norm for q in self.q]


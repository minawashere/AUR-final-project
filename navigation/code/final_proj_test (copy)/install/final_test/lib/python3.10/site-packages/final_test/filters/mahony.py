# mahony.py
import math

class MahonyAHRS:
    def __init__(self, sampleperiod=0.01, kp=2.0, ki=0.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.sampleperiod = sampleperiod
        self.q = [1.0, 0.0, 0.0, 0.0]  # Quaternion representation of sensor frame

        # Initialize error terms
        self.integral_fb = [0.0, 0.0, 0.0]

    def update_imu(self, gx, gy, gz, ax, ay, az):
        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return  # Handle NaN
        ax /= norm
        ay /= norm
        az /= norm

        # Calculate the quaternion rate of change
        q0, q1, q2, q3 = self.q
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Integrate to yield quaternion
        q0 += qDot1 * self.sampleperiod
        q1 += qDot2 * self.sampleperiod
        q2 += qDot3 * self.sampleperiod
        q3 += qDot4 * self.sampleperiod

        # Normalize the quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm == 0:
            return  # Handle NaN
        self.q = [q0 / norm, q1 / norm, q2 / norm, q3 / norm]

    @property
    def quaternion(self):
        return self.q

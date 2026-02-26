import numpy as np

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return control_signal


class SoftBodyController:
    def __init__(self, kp, ki, kd):
        self.deadband = 0.05

        # Four cavity pressures (order matches your description)
        self.p1 = 0.0  # upper left   (+x +z)
        self.p2 = 0.0  # upper right  (-x +z)
        self.p3 = 0.0  # lower left   (+x -z)
        self.p4 = 0.0  # lower right  (-x -z)

        # Single PID shared across all cavities
        self.pid = PID(kp, ki, kd)

        self.p_min = 0.0
        self.p_max = 0.0013

        self.files_written = 1
        self.e_mag_array = [];  self.dp_total_array = []
        self.p1_array = [];  self.p2_array = [];  self.p3_array = [];  self.p4_array = []

    def calculate_pressure(self, current, target, dt):
        # Axis errors
        error_x = target[0] - current[0]
        error_z = target[2] - current[2]

        # Four cavity "demands"
        e_Q1 = -error_x-error_z
        e_Q2 = +error_x-error_z
        e_Q3 = -error_x+error_z
        e_Q4 = +error_x+error_z

        errs = np.array([e_Q1, e_Q2, e_Q3, e_Q4], dtype=float)

        # Deadband per cavity
        errs[np.abs(errs) < self.deadband] = 0.0

        print(f"Errors:\t\t\t[{errs[0]:.5f}, {errs[1]:.5f}, {errs[2]:.5f}, {errs[3]:.5f}]")

        # If nothing to do, return current pressures
        if np.all(errs == 0.0):

            if self.files_written:
                file_e_mag = open('./e_mag.txt', 'w')
                for e_mag in self.e_mag_array:
                    file_e_mag.write(f'{e_mag:.8f}\n')
                file_e_mag.flush();  file_e_mag.close()

                file_dp_total = open('./dp_total.txt', 'w')
                for dp_total in self.dp_total_array:
                    file_dp_total.write(f'{dp_total:.8f}\n')
                file_dp_total.flush();  file_dp_total.close()

                file_Q1 = open('./p1.txt', 'w')
                for p1 in self.p1_array:
                    file_Q1.write(f'{p1:.8f}\n')
                file_Q1.flush();  file_Q1.close()

                file_Q2 = open('./p2.txt', 'w')
                for p2 in self.p2_array:
                    file_Q2.write(f'{p2:.8f}\n')
                file_Q2.flush();  file_Q2.close()

                file_Q3 = open('./p3.txt', 'w')
                for p3 in self.p3_array:
                    file_Q3.write(f'{p3:.8f}\n')
                file_Q3.flush();  file_Q3.close()

                file_Q4 = open('./p4.txt', 'w')
                for p4 in self.p4_array:
                    file_Q4.write(f'{p4:.8f}\n')
                file_Q4.flush();  file_Q4.close()

                self.files_written = 0

            return self.p1, self.p2, self.p3, self.p4

        # One PID update: use overall error magnitude as the single scalar error
        # (keeps ONE integral + derivative state, as requested)
        e_mag = float(np.linalg.norm(errs))
        dp_total = self.pid.update(e_mag, dt)

        self.e_mag_array.append(e_mag)
        self.dp_total_array.append(dp_total)

        # Distribute dp_total across cavities by signed relative demand.
        # Positive error -> increase that cavity; negative error -> decrease it.
        abs_sum = float(np.sum(np.abs(errs)))
        if abs_sum > 0:
            dp = dp_total * (errs / abs_sum)
        else:
            dp = np.zeros(4, dtype=float)

        # Update and clamp pressures
        self.p1 = float(np.clip(self.p1 + dp[3], self.p_min, self.p_max))
        self.p2 = float(np.clip(self.p2 + dp[2], self.p_min, self.p_max))
        self.p3 = float(np.clip(self.p3 + dp[1], self.p_min, self.p_max))
        self.p4 = float(np.clip(self.p4 + dp[0], self.p_min, self.p_max))

        self.p1_array.append(self.p1)
        self.p2_array.append(self.p2)
        self.p3_array.append(self.p3)
        self.p4_array.append(self.p4)

        return self.p1, self.p2, self.p3, self.p4

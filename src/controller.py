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
        self.p_ul = 0.0  # upper left   (-x +z)
        self.p_ur = 0.0  # upper right  (+x +z)
        self.p_ll = 0.0  # lower left   (-x -z)
        self.p_lr = 0.0  # lower right  (+x -z)

        # Single PID shared across all cavities
        self.pid = PID(kp, ki, kd)

        self.p_min = 0.0
        self.p_max = 0.0013

        self.files_written = 1
        self.e_mag_array = [];  self.dp_total_array = []
        self.p_ul_array = [];  self.p_ur_array = [];  self.p_ll_array = [];  self.p_lr_array = []

    def calculate_pressure(self, current, target, dt):
        # Axis errors
        error_x = target[0] - current[0]
        error_z = target[2] - current[2]

        # Four cavity "demands" (your required mapping)
        e_ul = -error_x + error_z
        e_ur = +error_x + error_z
        e_ll = -error_x - error_z
        e_lr = +error_x - error_z

        errs = np.array([e_ul, e_ur, e_ll, e_lr], dtype=float)

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

                file_p_ul = open('./p_ul.txt', 'w')
                for p_ul in self.p_ul_array:
                    file_p_ul.write(f'{p_ul:.8f}\n')
                file_p_ul.flush();  file_p_ul.close()

                file_p_ur = open('./p_ur.txt', 'w')
                for p_ur in self.p_ur_array:
                    file_p_ur.write(f'{p_ur:.8f}\n')
                file_p_ur.flush();  file_p_ur.close()

                file_p_ll = open('./p_ll.txt', 'w')
                for p_ll in self.p_ll_array:
                    file_p_ll.write(f'{p_ll:.8f}\n')
                file_p_ll.flush();  file_p_ll.close()

                file_p_lr = open('./p_lr.txt', 'w')
                for p_lr in self.p_lr_array:
                    file_p_lr.write(f'{p_lr:.8f}\n')
                file_p_lr.flush();  file_p_lr.close()

                self.files_written = 0

            return self.p_ul, self.p_ur, self.p_ll, self.p_lr

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
        self.p_ul = float(np.clip(self.p_ul + dp[0], self.p_min, self.p_max))
        self.p_ur = float(np.clip(self.p_ur + dp[1], self.p_min, self.p_max))
        self.p_ll = float(np.clip(self.p_ll + dp[2], self.p_min, self.p_max))
        self.p_lr = float(np.clip(self.p_lr + dp[3], self.p_min, self.p_max))

        self.p_ul_array.append(self.p_ul)
        self.p_ur_array.append(self.p_ur)
        self.p_ll_array.append(self.p_ll)
        self.p_lr_array.append(self.p_lr)

        return self.p_ul, self.p_ur, self.p_ll, self.p_lr

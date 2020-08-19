import numpy as np
import csv


class TorqueConverter:
    """
    液力变矩器模型
    """
    def __init__(self, config):
        self.config = config.torque_converter
        self.np = self.config.np
        self.nt = self.config.nt
        with open(self.config.K_data_path, 'r') as f:
            reader = csv.reader(f)
            self.K_table = np.array(list(reader), dtype=float)
        with open(self.config.Kp_data_path, 'r') as f:
            reader = csv.reader(f)
            self.Kp_table = np.array(list(reader), dtype=float)  # Kp的平方根

    def get_K_Kp(self):
        i = self.nt / self.np
        if i > self.K_table[-1,0]:
            K = self.K_table[-1,1]
        elif i < self.K_table[0, 0]:
            K = self.K_table[0, 1]
        else:
            K_id = np.where(self.K_table[:, 0] >= i)[0][0]
            if K_id == 0:
                K = self.K_table[0, 1]
            else:
                i1 = self.K_table[K_id - 1, 0]
                i2 = self.K_table[K_id, 0]
                K1 = self.K_table[K_id - 1, 1]
                K2 = self.K_table[K_id, 1]
                K = (i - i1) / (i2 - i1) * (K2 - K1) + K1

        if i > self.Kp_table[-1,0]:
            Kp = self.Kp_table[-1,1]
        elif i < self.Kp_table[0,0]:
            Kp = self.Kp_table[0,1]
        else:
            Kp_id = np.where(self.Kp_table[:, 0] >= i)[0][0]
            if Kp_id == 0:
                Kp = self.Kp_table[0, 1]
            else:
                i1 = self.Kp_table[Kp_id - 1, 0]
                i2 = self.Kp_table[Kp_id, 0]
                Kp1 = self.Kp_table[Kp_id - 1, 1]
                Kp2 = self.Kp_table[Kp_id, 1]
                Kp = (i - i1) / (i2 - i1) * (Kp2 - Kp1) + Kp1
        return K, Kp

    def get_torque(self):
        """
        :return: 泵轮转矩，涡轮转矩
        """
        K, Kp = self.get_K_Kp()
        if Kp < 0:
            Kp = -Kp**2
        else:
            Kp = Kp**2
        Tp = Kp*self.np**2
        Tt = K*Tp
        return Tp, Tt


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    tc = TorqueConverter(cfg)
    print(tc.get_torque())
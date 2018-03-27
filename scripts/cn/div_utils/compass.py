import numpy as np


class compass:
    cc = None
    NOISE = 0.002 # 2mG @ unit mag
    NOISE_PROD = (2 / (NOISE * NOISE))
    def __init__(self, cc = None):
        """
        Initialize the class with the given parameters.
        :param cc: Correction factors
        :return:
        """
        if cc is not None:
            self.cc = cc
        else:
            return

    def correct(self, mag):
        # code from Phidgets Compass Calibration source code
        ui = np.zeros((3, 1))
        Tui = np.zeros((3, 1))
        ui_Kronecker_Tui = np.zeros((9, 1))
        TtT = np.zeros((9, 1))
        mag_corrected = np.zeros((3, 1))
        
        ui[0] = mag[0] - self.cc[1]
        ui[1] = mag[1] - self.cc[2]
        ui[2] = mag[2] - self.cc[3]

        Tui[0] = self.cc[4] * ui[0] + self.cc[7] * ui[1] + self.cc[8] * ui[2]
        Tui[1] = self.cc[9] * ui[0] + self.cc[5] * ui[1] + self.cc[10] * ui[2]
        Tui[2] = self.cc[11] * ui[0] + self.cc[12] * ui[1] + self.cc[6] * ui[2]

        ui_Kronecker_Tui[0] = ui[0] * Tui[0]
        ui_Kronecker_Tui[1] = ui[0] * Tui[1]
        ui_Kronecker_Tui[2] = ui[0] * Tui[2]
        ui_Kronecker_Tui[3] = ui[1] * Tui[0]
        ui_Kronecker_Tui[4] = ui[1] * Tui[1]
        ui_Kronecker_Tui[5] = ui[1] * Tui[2]
        ui_Kronecker_Tui[6] = ui[2] * Tui[0]
        ui_Kronecker_Tui[7] = ui[2] * Tui[1]
        ui_Kronecker_Tui[8] = ui[2] * Tui[2]

        TtT[0] = self.cc[4] * self.cc[4] + self.cc[7] * self.cc[7] + self.cc[8] * self.cc[8]
        TtT[1] = self.cc[9] * self.cc[4] + self.cc[5] * self.cc[7] + self.cc[10] * self.cc[8]
        TtT[2] = self.cc[11] * self.cc[4] + self.cc[12] * self.cc[7] + self.cc[6] * self.cc[8]
        TtT[3] = self.cc[4] * self.cc[9] + self.cc[7] * self.cc[5] + self.cc[8] * self.cc[10]
        TtT[4] = self.cc[9] * self.cc[9] + self.cc[5] * self.cc[5] + self.cc[10] * self.cc[10]
        TtT[5] = self.cc[11] * self.cc[9] + self.cc[12] * self.cc[5] + self.cc[6] * self.cc[10]
        TtT[6] = self.cc[4] * self.cc[11] + self.cc[7] * self.cc[12] + self.cc[8] * self.cc[6]
        TtT[7] = self.cc[9] * self.cc[11] + self.cc[5] * self.cc[12] + self.cc[10] * self.cc[6]
        TtT[8] = self.cc[11] * self.cc[11] + self.cc[12] * self.cc[12] + self.cc[6] * self.cc[6]

        mag_corrected[0] = TtT[0] * ui[0] + TtT[3] * ui[1] + TtT[6] * ui[2]
        mag_corrected[1] = TtT[1] * ui[0] + TtT[4] * ui[1] + TtT[7] * ui[2]
        mag_corrected[2] = TtT[2] * ui[0] + TtT[5] * ui[1] + TtT[8] * ui[2]

        return mag_corrected


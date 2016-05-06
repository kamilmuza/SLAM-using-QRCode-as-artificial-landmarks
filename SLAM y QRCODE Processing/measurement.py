__author__ = 'Geist'
# Measurement models

# Beam model for laser ranger finders


#Sonar
class Sonar:
    def __init__(self, max_r, beta=15, t=0.5, max_occupied=0.98):
        self.max_r = max_r
        self.beta = beta
        self.t = t
        self.max_occ = max_occupied

    def prob_z_occ(self, r, alpha):
        return self.max_occ * ((self.max_r - r) / self.max_r + (self.beta - alpha) / self.beta) / 2

    def prob_z_free(self, r, alpha):
        return ((self.max_r - r) / self.max_r + (self.beta - alpha) / self.beta) / 2


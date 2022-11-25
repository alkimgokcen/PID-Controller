
import time
import os




class PID_Controller:

    #-------------------------------------------------------------------------------------
    # Alkim GOKCEN - University of Izmir Katip Celebi - alkim.gokcen@outlook.com
    # PID Control Library - Nov, 2022
    # This class is to employee the conventional PID controller method
    # u = Kp * e(t) + Ki * integral(e(t)) + Kd * d(e(t)) / dt
    # init the class => pidCotroller = PID_Controller(KP, Ki, Kd, sampleTime)
    # call PID as "pidController.PID(ActualOutput, DesiredOutput)"
    # P, PI, PID, PD, I, D, ID structures are configured
    # pidController.P, pidController.PI, pidController.PD
    # pidController.I, pidController.D, pidController.ID 
    #-------------------------------------------------------------------------------------
    #                               ALGORITHM
    # This algorithm solves the integral and derivative problems by employing
    # "Euler Method", and considers the error signal for current state.
    # e[n] = r[n] - y[n], where e is the tracking error, r is the reference
    # y is the plant output and n is the discrete time.
    # e[n] is considered to compute control signal u[n].
    # User may achieve y[n+1] by applying the u[n] to the plant.
    #-------------------------------------------------------------------------------------
    #                    Important Notes on Usage of The PID Controller
    # Please, do not forget to call controller algorithm appropriate to controller gains
    #-------------------------------------------------------------------------------------
    # V1.0.0
    #-------------------------------------------------------------------------------------
    def __init__(self, Kp, Ki, Kd, sampleTime):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sampleTime = sampleTime
        self.delayedTrackingError = 0
        self.integralError = 0
        
    def computeError(self, actualOutput, desiredOutput):
        trackingError = desiredOutput - actualOutput
        return trackingError
        
    def computeDerivative(self, trackingError):
        errorDot = trackingError - self.delayedTrackingError
        errorDot = errorDot / self.sampleTime
        self.delayedTrackingError = trackingError
        return errorDot
    
    def computeIntegral(self, trackingError):
        self.integralError = self.integralError + trackingError * self.sampleTime
        return self.integralError
    
    def PID(self, PlantOutput, reference):
        errorSignal = self.computeError(PlantOutput, reference)
        dError = self.computeDerivative(errorSignal)
        iError = self.computeIntegral(errorSignal)
        u = errorSignal * self.Kp + dError * self.Kd + iError * self.Ki
        return u, errorSignal, dError, iError
    
    def P(self, PlantOutput, reference):
        errorSignal = self.computeError(PlantOutput, reference)
        u = errorSignal * self.Kp 
        return u, errorSignal
    
    def I(self, PlantOutput, reference):
        errorSignal = self.computeError(PlantOutput, reference)
        iError = self.computeIntegral(errorSignal)
        u = errorSignal * self.Kp + iError * self.Ki
        return u, errorSignal, iError
    
    def D(self, PlantOutput, reference):
        errorSignal = self.computeError(PlantOutput, reference)
        dError = self.computeDerivative(errorSignal)
        u = self.Kd * dError
        return u, errorSignal, dError
    
    def PI(self, PlantOutput, reference):
        errorSignal = self.computeError(PlantOutput, reference)
        iError = self.computeIntegral(errorSignal)
        u = self.Kp * errorSignal + self.Ki * iError
        return u, errorSignal, iError
    
    def PD (self, PlantOutput, reference):
        errorSignal = self.computeError(PlantOutput, reference)
        dError = self.computeDerivative(errorSignal)
        u = self.Kp * dError + self.Kd * dError
        return u, errorSignal, dError
    
    def ID(self, PlantOutput, reference):
        errorSignal = self.computeError(PlantOutput, reference)
        dError = self.computeDerivative(errorSignal)
        iError = self.computeIntegral(errorSignal)
        u = self.Kd * dError + self.Ki * iError
        return u, errorSignal, dError, iError





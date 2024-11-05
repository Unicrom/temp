
import wpilib
from wpilib import (SmartDashboard, Compressor, Relay, PneumaticsModuleType ,)


import commands2
from constants import CannonConstant

class Cannon(commands2.Subsystem):
    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()

        self.cannonRelay = Relay(CannonConstant.kRelayAddress, wpilib._wpilib.Relay.Direction.kBothDirections)# PneumaticsModuleType.CTREPCM)
        self.cannonRelay.set(Relay.Value.kOff)
        SmartDashboard.putData("cannonRelay -from cannon", self.cannonRelay)

        turnoncommpressor = True # TODO: remove this gate
        if turnoncommpressor:
            self.compressor = Compressor(module=CannonConstant.kCompressorAddress,
                                         moduleType=PneumaticsModuleType.CTREPCM )
            # self.compressor.enableAnalog(65,75)#(105,115)
            SmartDashboard.putString("cannon status -from cannon: last action", "charging")
        # self.compressor.setClosedLoopControl(True)

    def periodic(self) -> None:
        # made default command -- self.cannonRelay.set(Relay.Value.kOn)#kOff)
        SmartDashboard.putString("cannon status -from cannon: last action", "stopped")
    
    def fire(self):
        self.cannonRelay.set(Relay.Value.kReverse)  #.kForward)
        SmartDashboard.putString("cannon status -from cannon: last action", "fireed")
    
    def stop(self):
        self.cannonRelay.set(Relay.Value.kOff)
        SmartDashboard.putString("cannon status -from cannon: last action", "stopped")

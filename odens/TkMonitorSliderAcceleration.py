#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-
# Original: OpenRTM-aist/1.1.2/Components/Python/Examples/Slider_and_Motor/SliderComp.py

import sys
import time
import math
sys.path.append(".")

import RTC
import OpenRTM_aist

import slider

### Windowsのpythowで標準出力に出力し続けるとIOErrorになる問題を回避
import os
if sys.executable.endswith("pythonw.exe"):
  devnull = file(os.devnull, 'w')
  sys.stdout = sys.stderr = devnull
###

channels = (
  ("x",  -10.0,  10.0, 0.1, 200),
  ("y",  -10.0,  10.0, 0.1, 200),
  ("z",  -10.0,  10.0, 0.1, 200))

mod_spec = ["implementation_id", "TkMonitorSliderAcceleration", 
            "type_name", "TkMonitorSliderAcceleration", 
            "description", "", 
            "version", "1.0", 
            "vendor", "MasutaniLab", 
            "category", "Generic", 
            "activity_type", "DataFlowComponent", 
            "max_instance", "10", 
            "language", "Python", 
            "lang_type""SCRIPT",
            ""]

sl = slider.SliderMulti(channels, varType="double",
                        title = "Acceleration")

class TkMonitorSliderAcceleration(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    self._num = len(channels)
    return
        

  def onInitialize(self):
    print("onInitialize()")
    self._value_data = RTC.Acceleration3D(0,0,0)
    self._valueIn = OpenRTM_aist.InPort("value", self._value_data)

    self.addInPort("value", self._valueIn)

    return RTC.RTC_OK

  def onActivated(self, ec_id):
    print("onActivated()")
    return RTC.RTC_OK


  def onExecute(self, ec_id):
    if self._valueIn.isNew():
      try:
        indata = self._valueIn.read()
        val = [indata.ax, indata.ay, indata.az]
        #print(val)
        if len(val) == self._num:
          sl.set(val)
      except Exception as e:
        print("Exception cought in onExecute()")
        print(e)

    return RTC.RTC_OK
  

  def onShutdown(self, ec_id):
    print("onShutdown()")
    sl.quit()
    return RTC.RTC_OK



def TkMonitorSliderAccelerationInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=mod_spec)
  manager.registerFactory(profile,
                          TkMonitorSliderAcceleration,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  TkMonitorSliderAccelerationInit(manager)

  # Create a component
  comp = manager.createComponent("TkMonitorSliderAcceleration")

  print("Componet created")


def main():
  # Initialize manager
  mgr = OpenRTM_aist.Manager.init(sys.argv)

  # Set module initialization proceduer
  # This procedure will be invoked in activateManager() function.
  mgr.setModuleInitProc(MyModuleInit)

  # Activate manager and register to naming service
  mgr.activateManager()

  # run the manager in blocking mode
  # runManager(False) is the default
  #mgr.runManager()

  # If you want to run the manager in non-blocking mode, do like this
  mgr.runManager(True)
  sl.mainloop()

if __name__ == "__main__":
  main()

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-
# Original: OpenRTM-aist/1.1.2/Components/Python/Examples/Slider_and_Motor/SliderComp.py

import sys
import time
sys.path.append(".")

import RTC
import OpenRTM_aist
import math

import slider

### Windowsのpythowで標準出力に出力し続けるとIOErrorになる問題を回避
import os
if sys.executable.endswith("pythonw.exe"):
  devnull = file(os.devnull, 'w')
  sys.stdout = sys.stderr = devnull
###

d2r = math.radians(1)
channels = (
  ("MFRAME",  -687.5,  687.5, 1, 200, 0, d2r),
  ("BLOCK",   -48.0,   69.5, 1, 200, 0, d2r),
  ("BOOM",   -75.1,   53.2, 1, 200, -70.0, d2r),
  ("ARM",    30.7,  152.1, 1, 200, 150.0, d2r),
  ("TOHKU_PITCH",   -33.7,   90.0, 1, 200, 0, d2r),
  ("TOHKU_ROLL",  -180.0,  180.0, 1, 200, 0, d2r),
  ("TIP_01",   -20.0,    0.0, 1, 200, 0, d2r), #"TOHKU_TIP_01"
  ("TIP_02",   -20.0,    0.0, 1, 200, 0, d2r), #"TOHKU_TIP_02"
  ("UFRAME",  -687.5,  687.5, 1, 200, 0, d2r),
  ("MNP_SWING",   -60.0,   60.0, 1, 200, 0, d2r),
  ("MANIBOOM",     0.0,  115.0, 1, 200, 0, d2r),
  ("MANIARM",  -110.0,    0.0, 1, 200, 0, d2r),
  ("MANIELBOW",   -90.0,   20.0, 1, 200, 0, d2r),
  ("YAWJOINT",   -50.48,  50.48, 1, 200, 0, d2r),
  ("HANDBASE",  -270.0,  270.0, 1, 200, 0, d2r),
  ("PUSHROD",    -0.0507,  0.0, 0.0001, 200, 0, 1))

mod_spec = ["implementation_id", "TkSlider", 
            "type_name", "TkSlider", 
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
                        title = "Command")

class TkSlider(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return
        

  def onInitialize(self):
    print("onInitialize()")
    self._sl_data = RTC.TimedDoubleSeq(RTC.Time(0,0), [])
    self._slOut = OpenRTM_aist.OutPort("slider", self._sl_data)

    self.addOutPort("slider", self._slOut)
    return RTC.RTC_OK

  def onActivated(self, ec_id):
    print("onActivated()")
    self._prev_data = sl.get()
    #print(self._prev_data)
    time.sleep(0.01)
    return RTC.RTC_OK


  def onExecute(self, ec_id):
    try:
      self._sl_data.data = sl.get()
      if self._sl_data.data != self._prev_data:
        #print self._sl_data.data
        self._slOut.write()
      self._prev_data = self._sl_data.data
    except Exception as e:
      print("Exception cought in onExecute()")
      print(e)
    time.sleep(0.01)
    return RTC.RTC_OK


  def onShutdown(self, ec_id):
    print("onShutdown()")
    sl.quit()
    return RTC.RTC_OK



def TkSliderInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=mod_spec)
  manager.registerFactory(profile,
                          TkSlider,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  TkSliderInit(manager)

  # Create a component
  comp = manager.createComponent("TkSlider")

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

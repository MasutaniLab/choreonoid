import WRSUtil
WRSUtil.loadProject(
    "SingleSceneView", "T4", [ "AGXSimulator", "AISTSimulator" ], "Quadcopter",
    enableMulticopterSimulation = True, enableVisionSimulation = True, targetVisionSensors = "", remoteType = "RTM")

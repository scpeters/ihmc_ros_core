package us.ihmc.ros.java_adapter

import us.ihmc.ros.java_adapter.yaml.IHMCROSJavaYamlLoader

/**
 * Created by parallels on 3/15/16.
 */
class IHMCROSJavaYamlTester {
    public static void main(String[] args) {
        def tmpDeps = new IHMCROSJavaDependencyHolder(new ArrayList<String>())
        tmpDeps.dependencies.add("us.ihmc:Atlas:0.7.4")

        def tmpVmConfig = new IHMCROSJavaVMConfiguration("us.ihmc.atlas.AtlasFlatGroundWalkingTrack", "4g")

//        def tmpArgs = new ArrayList()
//        tmpArgs.add("-m")
//        tmpArgs.add("ATLAS_V5_UNPLUGGED_NO_HANDS")

        def config = new IHMCROSJavaConfiguration(tmpDeps, tmpVmConfig, new ArrayList<String>())


        def loader = new IHMCROSJavaYamlLoader()
        loader.dumpYaml(config, "/home/parallels/dump2.yaml")

        loader.loadYaml("/home/parallels/dump2.yaml")
    }
}
